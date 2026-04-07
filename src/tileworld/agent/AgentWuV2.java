package tileworld.agent;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

import sim.field.grid.ObjectGrid2D;
import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.agent.classdefines.MemoryObjectType;
import tileworld.agent.classdefines.MemorySideCard;
import tileworld.agent.classdefines.MemorySideCardEntry;
import tileworld.agent.utils.SensorSnapshotCodec;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.environment.TWObstacle;

/**
 * AgentWuV2 - 基于 AgentWu 的改进版本
 *
 * 主要修改：
 * 1. 路径规划：保留 A* + 缓存，移除贪心降级方案
 * 2. 目标选择/碰撞检测/冲突避免：基于本地感知 + 共同记忆
 * 3. 每15步检测目标有效性，不存在或被锁定则重新规划
 * 4. 探索策略：始终使用蛇形探索（不使用 Z 字型）
 */
public class AgentWuV2 extends Group7AgentBase {

    // ========== Constants ==========
    private static final int FUEL_FORCE_REFUEL = 150;
    private static final int FUEL_WARNING = 200;
    private static final int FUEL_MARGIN = 32;
    private static final int LOCAL_RANGE = 20;
    private static final int ASTAR_MAX_NODES = 500;
    private static final int LOCK_TTL_MIN_STEPS = 8;
    private static final int LOCK_TTL_MAX_STEPS = 20;
    private static final int LOCK_RENEW_BEFORE_STEPS = 2;
    private static final double LOCK_PRIORITY_EPSILON = 1e-9;
    private static final int TARGET_CHECK_INTERVAL = 15;
    private static final int MAX_PATH_PLANNING_RETRY = 3;

    // ========== State ==========
    private final MemorySideCard sideCard;
    private final Map<String, TargetLease> teammateTargetLeases;
    private final int ownTargetLockTtlSteps;

    // Owned target lock
    private Int2D lockedTargetCell;
    private MemoryObjectType lockedTargetType;
    private double lockedTargetPriority;
    private long lockedTargetStep;
    private long lockedTargetExpiryStep;

    // Path caching
    private List<TWDirection> cachedPath;
    private Int2D cachedTarget;

    // Serpentine exploration
    private boolean serpentineRight;
    private boolean serpentineDown;
    private int serpentineX;
    private int serpentineY;
    private int serpentineStep;

    // Target validation tracking
    private long lastTargetCheckStep;
    private Int2D lastValidatedTarget;

    // Path planning retry tracking
    private int pathPlanningRetryCount;
    private Int2D lastRetryTarget;

    // ========== Constructor ==========
    public AgentWuV2(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);

        this.sideCard = new MemorySideCard();
        this.teammateTargetLeases = new HashMap<>();
        this.ownTargetLockTtlSteps = computeDefaultTargetLockTtl(env);

        this.lockedTargetCell = null;
        this.lockedTargetType = null;
        this.lockedTargetPriority = 0.0;
        this.lockedTargetStep = -1L;
        this.lockedTargetExpiryStep = -1L;

        this.cachedPath = null;
        this.cachedTarget = null;

        // Initialize serpentine state
        this.serpentineRight = true;
        this.serpentineDown = true;
        this.serpentineX = xpos;
        this.serpentineY = ypos;
        this.serpentineStep = Math.max(1, (Parameters.defaultSensorRange * 2) + 1);

        // Initialize target validation state
        this.lastTargetCheckStep = -1;
        this.lastValidatedTarget = null;

        // Initialize path planning retry state
        this.pathPlanningRetryCount = 0;
        this.lastRetryTarget = null;

        enableZoneCoordination();
    }

    // ========== Required Override Methods ==========
    @Override
    public String getName() {
        return super.getName();
    }

    @Override
    protected TWThought think() {
        long step = currentStep();

        rememberFuelStationsInSensorRange();
        processIncomingMessages(step);
        processZoneCoordinationInThink();
        pruneStaleTeammateTargetLeases(step);
        ensureActiveLockStillValid(step);
        synchronizeSideCardWithCurrentObservation(step);

        // Check zone coordination status
        // 始终使用蛇形探索（不区分分区前后）
        if (!isZoneCoordinationComplete()) {
            if (findFuelStationInMemory() != null) {
                markZoneCoordinationComplete();
            } else {
                releaseOwnTargetLock("zone_bootstrap");
                return serpentineExplore();
            }
        }

        // Force refuel when fuel < 150
        if (getFuelLevel() < FUEL_FORCE_REFUEL) {
            if (this.getEnvironment().inFuelStation(this)) {
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            }
            return thinkRefuel();
        }

        // Warning mode: fuel < 200
        boolean warningMode = getFuelLevel() < FUEL_WARNING;

        // Check immediate actions at current position
        TWEntity here = (TWEntity) this.getEnvironment().getObjectGrid().get(getX(), getY());
        if (here instanceof TWHole && this.hasTile()) {
            if (!isOwnLockAt(MemoryObjectType.HOLE, getX(), getY())) {
                releaseOwnTargetLock("putdown_other_target");
            }
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (here instanceof TWTile && this.carriedTiles.size() < 3) {
            if (!isOwnLockAt(MemoryObjectType.TILE, getX(), getY())) {
                releaseOwnTargetLock("pickup_other_target");
            }
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        // 每15步或目标变化时，检测目标有效性
        if (shouldCheckTargetValidity(step)) {
            if (!validateCurrentTarget(step)) {
                clearCachedPath();
            }
        }

        // Check sensor range + shared memory for tiles/holes (基于共同记忆的目标选择)
        TWTile nearestTile = findNearestTileFromAllMemory();
        TWHole nearestHole = findNearestHoleFromAllMemory();

        if (nearestTile != null && this.carriedTiles.size() < 3) {
            int tx = nearestTile.getX();
            int ty = nearestTile.getY();
            // 目标冲突避免：检查是否被队友锁定
            if (!isCellLockedByTeammate(tx, ty)) {
                if (warningMode && manhattan(getX(), getY(), tx, ty) > 15) {
                    return thinkRefuel();
                }
                if (hasEnoughFuelForRoundTrip(tx, ty)) {
                    if (tryAcquireOrKeepTargetLockForTile(tx, ty, step)) {
                        resetPathPlanningRetry();  // 选择新目标时重置重试计数
                        return planToTarget(tx, ty);
                    }
                } else {
                    return thinkRefuel();
                }
            }
        }

        if (nearestHole != null && this.hasTile()) {
            int hx = nearestHole.getX();
            int hy = nearestHole.getY();
            // 目标冲突避免：检查是否被队友锁定
            if (!isCellLockedByTeammate(hx, hy)) {
                if (warningMode && manhattan(getX(), getY(), hx, hy) > 15) {
                    return thinkRefuel();
                }
                if (hasEnoughFuelForRoundTrip(hx, hy)) {
                    if (tryAcquireOrKeepTargetLockForHole(hx, hy, step)) {
                        resetPathPlanningRetry();  // 选择新目标时重置重试计数
                        return planToTarget(hx, hy);
                    }
                } else {
                    return thinkRefuel();
                }
            }
        }

        // Warning mode: refuel if fuel station visible
        if (warningMode) {
            TWFuelStation fuel = (TWFuelStation) memory.getClosestObjectInSensorRange(TWFuelStation.class);
            if (fuel != null) {
                return thinkRefuel();
            }
        }

        // 始终使用蛇形探索（不区分分区前后）
        releaseOwnTargetLock("explore_no_candidate");
        return serpentineExplore();
    }

    @Override
    protected void act(TWThought thought) {
        if (thought == null) {
            return;
        }

        try {
            switch (thought.getAction()) {
                case MOVE:
                    this.move(thought.getDirection());
                    // Invalidate cache if position changed unexpectedly
                    if (cachedTarget != null && getX() == cachedTarget.x && getY() == cachedTarget.y) {
                        clearCachedPath();
                    }
                    break;
                case PICKUP:
                    TWEntity obj = (TWEntity) this.getEnvironment().getObjectGrid().get(getX(), getY());
                    if (obj instanceof TWTile && this.carriedTiles.size() < 3) {
                        int tx = obj.getX();
                        int ty = obj.getY();
                        this.pickUpTile((TWTile) obj);
                        sideCard.remove(MemoryObjectType.TILE, tx, ty);
                        this.memory.removeAgentPercept(tx, ty);
                        publishActionEvent(CommType.ACTION_PICKUP_TILE, tx, ty, "");
                        clearCachedPath();
                    }
                    break;
                case PUTDOWN:
                    TWEntity hole = (TWEntity) this.getEnvironment().getObjectGrid().get(getX(), getY());
                    if (hole instanceof TWHole && this.hasTile()) {
                        int hx = hole.getX();
                        int hy = hole.getY();
                        this.putTileInHole((TWHole) hole);
                        sideCard.remove(MemoryObjectType.HOLE, hx, hy);
                        this.memory.removeAgentPercept(hx, hy);
                        publishActionEvent(CommType.ACTION_FILL_HOLE, hx, hy, "");
                        clearCachedPath();
                    }
                    break;
                case REFUEL:
                    if (this.getEnvironment().inFuelStation(this)) {
                        this.refuel();
                    }
                    break;
                default:
                    break;
            }
        } catch (Exception e) {
            clearCachedPath();
        }
    }

    @Override
    protected void appendCustomMessages(List<Message> outbox) {
        long step = currentStep();
        if (hasActiveTargetLock() && shouldRenewTargetLock(step)) {
            outbox.add(createProtocolMessage(
                    CommType.TARGET_LOCK,
                    lockedTargetCell.x,
                    lockedTargetCell.y,
                    encodeTargetLockPayload(lockedTargetPriority, ownTargetLockTtlSteps)));
            lockedTargetStep = step;
            lockedTargetExpiryStep = step + ownTargetLockTtlSteps;
        }
    }

    // ========== Message Processing ==========
    private void processIncomingMessages(long step) {
        List<Message> inbox = new ArrayList<>(this.getEnvironment().getMessages());
        for (Message message : inbox) {
            if (message == null) continue;

            String directFrom = message.getFrom();
            if (directFrom != null && directFrom.equals(this.getName())) continue;

            String directTo = message.getTo();
            if (directTo != null && !directTo.equals("ALL") && !directTo.equals(this.getName())) continue;

            ParsedMessage parsed = parseProtocolMessage(message);
            if (parsed == null) continue;

            if (parsed.from == null || parsed.from.equals(this.getName())) continue;
            if (!isMessageForMe(parsed)) continue;

            switch (parsed.type) {
                case OBS_NEW_TILE:
                    handleTeammateObjectObservation(MemoryObjectType.TILE, parsed.x, parsed.y, parsed.step);
                    break;
                case OBS_NEW_HOLE:
                    handleTeammateObjectObservation(MemoryObjectType.HOLE, parsed.x, parsed.y, parsed.step);
                    break;
                case OBS_OBSTACLE:
                    handleTeammateObjectObservation(MemoryObjectType.OBSTACLE, parsed.x, parsed.y, parsed.step);
                    break;
                case OBS_FUEL_ONCE:
                    rememberFuelStationFromMessage(parsed.x, parsed.y);
                    break;
                case OBS_SENSOR_SNAPSHOT:
                    processSnapshotMessage(parsed);
                    break;
                case TARGET_LOCK:
                    handleTeammateTargetLock(parsed);
                    break;
                case TARGET_RELEASE:
                    handleTeammateTargetRelease(parsed);
                    break;
                case ACTION_PICKUP_TILE:
                    sideCard.remove(MemoryObjectType.TILE, parsed.x, parsed.y);
                    this.memory.removeAgentPercept(parsed.x, parsed.y);
                    break;
                case ACTION_FILL_HOLE:
                    sideCard.remove(MemoryObjectType.HOLE, parsed.x, parsed.y);
                    this.memory.removeAgentPercept(parsed.x, parsed.y);
                    break;
                default:
                    break;
            }
        }
    }

    private void handleTeammateObjectObservation(MemoryObjectType type, int x, int y, long observedAt) {
        sideCard.upsert(type, x, y, observedAt, false);
        TWEntity entity = createGhostEntity(type, x, y, observedAt);
        if (entity != null) {
            this.memory.getMemoryGrid().set(x, y, entity);
        }
    }

    private void processSnapshotMessage(ParsedMessage parsed) {
        List<SensorSnapshotCodec.SnapshotItem> items = SensorSnapshotCodec.decode(parsed.payload);
        for (SensorSnapshotCodec.SnapshotItem item : items) {
            if ("E".equals(item.code)) {
                for (MemoryObjectType type : MemoryObjectType.values()) {
                    sideCard.remove(type, item.x, item.y);
                }
                // 从共同记忆（环境记忆）中删除
                this.memory.getMemoryGrid().set(item.x, item.y, null);
                // 从感知记忆中删除（如果该格子在感知范围内）
                removeFromSensorMemoryIfInRange(item.x, item.y);
                continue;
            }

            MemoryObjectType type = MemoryObjectType.fromSnapshotCode(item.code);
            if (type != null) {
                sideCard.upsert(type, item.x, item.y, parsed.step, false);
                TWEntity entity = createGhostEntity(type, item.x, item.y, parsed.step);
                if (entity != null) {
                    this.memory.getMemoryGrid().set(item.x, item.y, entity);
                }
            }
        }
    }

    /**
     * 从感知记忆中删除指定格子
     * 检查该格子是否在当前感知范围内，如果在则从感知记忆中删除
     */
    private void removeFromSensorMemoryIfInRange(int x, int y) {
        int sensorRange = Parameters.defaultSensorRange;
        int dx = Math.abs(x - getX());
        int dy = Math.abs(y - getY());
        if (dx <= sensorRange && dy <= sensorRange) {
            // 从感知记忆中删除（设置为 null）
            this.getEnvironment().getObjectGrid().set(x, y, null);
        }
    }

    private TWEntity createGhostEntity(MemoryObjectType type, int x, int y, double observedAt) {
        double death = observedAt + Parameters.lifeTime;
        switch (type) {
            case TILE:
                return new TWTile(x, y, this.getEnvironment(), observedAt, death);
            case HOLE:
                return new TWHole(x, y, this.getEnvironment(), observedAt, death);
            case OBSTACLE:
                return new TWObstacle(x, y, this.getEnvironment(), observedAt, death);
            default:
                return null;
        }
    }

    private void rememberFuelStationFromMessage(int x, int y) {
        ObjectGrid2D memoryGrid = this.memory.getMemoryGrid();
        if (!(memoryGrid.get(x, y) instanceof TWFuelStation)) {
            memoryGrid.set(x, y, new TWFuelStation(x, y, this.getEnvironment()));
        }
    }

    // ========== SideCard Synchronization ==========
    private void synchronizeSideCardWithCurrentObservation(long step) {
        ObjectGrid2D grid = this.getEnvironment().getObjectGrid();
        int range = Parameters.defaultSensorRange;

        for (int dx = -range; dx <= range; dx++) {
            for (int dy = -range; dy <= range; dy++) {
                int x = getX() + dx;
                int y = getY() + dy;
                if (!this.getEnvironment().isInBounds(x, y)) continue;

                TWEntity obj = (TWEntity) grid.get(x, y);
                if (obj == null) {
                    for (MemoryObjectType type : MemoryObjectType.values()) {
                        sideCard.remove(type, x, y);
                    }
                    continue;
                }

                if (obj instanceof TWFuelStation) {
                    this.memory.getMemoryGrid().set(x, y, obj);
                    continue;
                }

                MemoryObjectType type = MemoryObjectType.fromEntity(obj);
                if (type != null) {
                    sideCard.upsert(type, x, y, step, true);
                    this.memory.getMemoryGrid().set(x, y, obj);
                }
            }
        }
    }

    // ========== Target Lock Collaboration ==========
    private void handleTeammateTargetLock(ParsedMessage parsed) {
        Double priority = parseTargetLockPriority(parsed.payload);
        if (priority == null) return;

        Integer ttl = parseTargetLockTtl(parsed.payload);
        int ttlSteps = (ttl == null) ? ownTargetLockTtlSteps : ttl.intValue();

        String key = MemorySideCard.cellKey(parsed.x, parsed.y);
        TargetLease incoming = new TargetLease(parsed.from, priority.doubleValue(), parsed.step, ttlSteps);
        TargetLease existing = teammateTargetLeases.get(key);

        if (existing == null) {
            teammateTargetLeases.put(key, incoming);
            return;
        }

        if (existing.owner.equals(incoming.owner)) {
            if (incoming.step >= existing.step) {
                teammateTargetLeases.put(key, incoming);
            }
            return;
        }

        if (isLeasePreferred(incoming, existing)) {
            teammateTargetLeases.put(key, incoming);
        }
    }

    private void handleTeammateTargetRelease(ParsedMessage parsed) {
        String key = MemorySideCard.cellKey(parsed.x, parsed.y);
        TargetLease existing = teammateTargetLeases.get(key);
        if (existing != null && existing.owner.equals(parsed.from)) {
            teammateTargetLeases.remove(key);
        }
    }

    private void pruneStaleTeammateTargetLeases(long step) {
        List<String> staleKeys = new ArrayList<>();
        for (Map.Entry<String, TargetLease> entry : teammateTargetLeases.entrySet()) {
            if (step > entry.getValue().expiresAtStep) {
                staleKeys.add(entry.getKey());
            }
        }
        teammateTargetLeases.keySet().removeAll(staleKeys);
    }

    private void ensureActiveLockStillValid(long step) {
        if (!hasActiveTargetLock()) return;

        if (step > lockedTargetExpiryStep) {
            releaseOwnTargetLock("local_lease_expired");
            return;
        }

        TargetLease teammate = teammateTargetLeases.get(MemorySideCard.cellKey(lockedTargetCell.x, lockedTargetCell.y));
        if (teammate != null) {
            if (!isPreferredLock(getName(), lockedTargetPriority, lockedTargetStep,
                    teammate.owner, teammate.priority, teammate.step)) {
                releaseOwnTargetLock("lost_priority");
                return;
            }
        }
    }

    private boolean tryAcquireOrKeepTargetLockForTile(int x, int y, long step) {
        if (isOwnLockAt(MemoryObjectType.TILE, x, y)) return true;

        double priority = computeLockPriority(MemoryObjectType.TILE, x, y, step);
        if (isBlockedByPreferredTeammateLock(x, y, priority, step)) return false;

        releaseOwnTargetLock("switch_target");
        lockedTargetCell = new Int2D(x, y);
        lockedTargetType = MemoryObjectType.TILE;
        lockedTargetPriority = priority;
        lockedTargetStep = step;
        lockedTargetExpiryStep = step + ownTargetLockTtlSteps;
        publishProtocolMessage(CommType.TARGET_LOCK, x, y, encodeTargetLockPayload(priority, ownTargetLockTtlSteps));
        return true;
    }

    private boolean tryAcquireOrKeepTargetLockForHole(int x, int y, long step) {
        if (isOwnLockAt(MemoryObjectType.HOLE, x, y)) return true;

        double priority = computeLockPriority(MemoryObjectType.HOLE, x, y, step);
        if (isBlockedByPreferredTeammateLock(x, y, priority, step)) return false;

        releaseOwnTargetLock("switch_target");
        lockedTargetCell = new Int2D(x, y);
        lockedTargetType = MemoryObjectType.HOLE;
        lockedTargetPriority = priority;
        lockedTargetStep = step;
        lockedTargetExpiryStep = step + ownTargetLockTtlSteps;
        publishProtocolMessage(CommType.TARGET_LOCK, x, y, encodeTargetLockPayload(priority, ownTargetLockTtlSteps));
        return true;
    }

    private boolean isBlockedByPreferredTeammateLock(int x, int y, double myPriority, long myStep) {
        TargetLease teammate = teammateTargetLeases.get(MemorySideCard.cellKey(x, y));
        if (teammate == null) return false;
        return !isPreferredLock(getName(), myPriority, myStep, teammate.owner, teammate.priority, teammate.step);
    }

    private boolean isLeasePreferred(TargetLease incoming, TargetLease existing) {
        return isPreferredLock(incoming.owner, incoming.priority, incoming.step,
                existing.owner, existing.priority, existing.step);
    }

    private boolean isPreferredLock(String ownerA, double priorityA, long stepA,
                                   String ownerB, double priorityB, long stepB) {
        if (Math.abs(priorityA - priorityB) > LOCK_PRIORITY_EPSILON) {
            return priorityA > priorityB;
        }
        if (stepA != stepB) {
            return stepA < stepB;
        }
        String safeA = (ownerA == null) ? "" : ownerA;
        String safeB = (ownerB == null) ? "" : ownerB;
        return safeA.compareTo(safeB) < 0;
    }

    private double computeLockPriority(MemoryObjectType type, int x, int y, long step) {
        long seed = 1469598103934665603L;
        seed ^= (long) getName().hashCode();
        seed *= 1099511628211L;
        seed ^= (long) type.ordinal();
        seed *= 1099511628211L;
        seed ^= (long) x;
        seed *= 1099511628211L;
        seed ^= (long) y;
        seed *= 1099511628211L;
        seed ^= step;
        seed ^= (seed >>> 33);
        seed *= 0xff51afd7ed558ccdL;
        seed ^= (seed >>> 33);
        seed *= 0xc4ceb9fe1a85ec53L;
        seed ^= (seed >>> 33);
        return (seed & Long.MAX_VALUE) % 1000000L / 1000000.0;
    }

    private static int computeDefaultTargetLockTtl(TWEnvironment env) {
        int span = Math.max(1, env.getxDimension() + env.getyDimension());
        int ttl = span / 6;
        return Math.max(LOCK_TTL_MIN_STEPS, Math.min(LOCK_TTL_MAX_STEPS, ttl));
    }

    private boolean hasActiveTargetLock() {
        return lockedTargetCell != null && lockedTargetType != null;
    }

    private boolean shouldRenewTargetLock(long step) {
        if (!hasActiveTargetLock()) return false;
        if (lockedTargetExpiryStep < 0) return true;
        return step >= (lockedTargetExpiryStep - LOCK_RENEW_BEFORE_STEPS);
    }

    private boolean isOwnLockAt(MemoryObjectType type, int x, int y) {
        return hasActiveTargetLock()
                && lockedTargetType == type
                && lockedTargetCell.x == x
                && lockedTargetCell.y == y;
    }

    private void releaseOwnTargetLock(String reason) {
        if (!hasActiveTargetLock()) return;
        publishProtocolMessage(CommType.TARGET_RELEASE, lockedTargetCell.x, lockedTargetCell.y, reason != null ? reason : "");
        lockedTargetCell = null;
        lockedTargetType = null;
        lockedTargetPriority = 0.0;
        lockedTargetStep = -1L;
        lockedTargetExpiryStep = -1L;
    }

    private boolean isCellLockedByTeammate(int x, int y) {
        return teammateTargetLeases.containsKey(MemorySideCard.cellKey(x, y));
    }

    // ========== Exploration (逐行蛇形 + 分区限制) ==========
    /**
     * 逐行蛇形探索，支持分区边界限制和反弹机制
     * - 未找到加油站前：严格在分配的分区内行走
     * - 到达边界后反弹，而不是重置
     * - 逐行走（步长=1），确保覆盖每个格子
     */
    private TWThought serpentineExplore() {
        // 确定探索边界：有分区则用分区边界，否则用全局地图边界
        int minX, maxX, minY, maxY;
        if (!isZoneCoordinationComplete() && hasZoneAssignment()) {
            ZoneAssignment zone = getZoneAssignment();
            minX = zone.x1;
            maxX = zone.x2;
            minY = zone.y1;
            maxY = zone.y2;
        } else {
            minX = 0;
            maxX = this.getEnvironment().getxDimension() - 1;
            minY = 0;
            maxY = this.getEnvironment().getyDimension() - 1;
        }

        // 更新蛇形目标
        if (serpentineRight) {
            int nextX = serpentineX + serpentineStep;
            if (nextX <= maxX) {
                serpentineX = nextX;
            } else {
                // 到达右边界，反弹向上或下移动一行
                serpentineX = maxX;
                serpentineY = nextSerpentineY(serpentineY, minY, maxY);
                serpentineRight = false;
            }
        } else {
            int nextX = serpentineX - serpentineStep;
            if (nextX >= minX) {
                serpentineX = nextX;
            } else {
                // 到达左边界，反弹向上或下移动一行
                serpentineX = minX;
                serpentineY = nextSerpentineY(serpentineY, minY, maxY);
                serpentineRight = true;
            }
        }

        // 确保在边界内
        serpentineY = Math.max(minY, Math.min(maxY, serpentineY));

        Int2D target = new Int2D(serpentineX, serpentineY);
        return planToTarget(target.x, target.y);
    }

    /**
     * 计算蛇形探索的下一个Y坐标（带反弹机制）
     */
    private int nextSerpentineY(int currentY, int minY, int maxY) {
        int step = serpentineStep;
        int nextY = serpentineDown ? (currentY + step) : (currentY - step);

        // 检查是否在有效范围内
        if (nextY >= minY && nextY <= maxY) {
            return nextY;
        }

        // 超出边界，反弹切换方向
        serpentineDown = !serpentineDown;
        step = serpentineDown ? this.serpentineStep : -this.serpentineStep;
        nextY = currentY + step;

        // 修正到边界内
        if (nextY < minY) {
            return minY;
        } else if (nextY > maxY) {
            return maxY;
        }
        return nextY;
    }

    // ========== Refuel ==========
    private TWThought thinkRefuel() {
        TWFuelStation station = (TWFuelStation) memory.getClosestObjectInSensorRange(TWFuelStation.class);
        if (station != null) {
            return planToTarget(station.getX(), station.getY());
        }
        station = findFuelStationInMemory();
        if (station != null) {
            return planToTarget(station.getX(), station.getY());
        }
        return waitThought();
    }

    private boolean hasEnoughFuelForRoundTrip(int tx, int ty) {
        int dist = manhattan(getX(), getY(), tx, ty);
        return getFuelLevel() >= dist * 2 + FUEL_MARGIN;
    }

    // ========== 基于共同记忆的目标选择（带区域限制） ==========
    /**
     * 从本地感知 + 共同记忆中查找最近的 Tile（限制在分配区域内）
     */
    private TWTile findNearestTileFromAllMemory() {
        TWTile best = null;
        int bestDist = Integer.MAX_VALUE;
        int cx = getX();
        int cy = getY();
        ZoneAssignment zone = hasZoneAssignment() ? getZoneAssignment() : null;

        // 1. 先检查本地感知范围内的 Tile（如果不在区域内则跳过）
        ObjectGrid2D grid = this.getEnvironment().getObjectGrid();
        int range = Parameters.defaultSensorRange;
        for (int dx = -range; dx <= range; dx++) {
            for (int dy = -range; dy <= range; dy++) {
                int x = cx + dx;
                int y = cy + dy;
                if (!this.getEnvironment().isInBounds(x, y)) continue;
                // 区域边界检查
                if (!isInAssignedZone(x, y, zone)) continue;

                TWEntity obj = (TWEntity) grid.get(x, y);
                if (obj instanceof TWTile) {
                    int dist = manhattan(cx, cy, x, y);
                    if (dist < bestDist) {
                        bestDist = dist;
                        best = (TWTile) obj;
                    }
                }
            }
        }

        // 2. 检查共同记忆中的 Tile (sideCard)（限制在区域内）
        List<MemorySideCardEntry> tileEntries = sideCard.listByType(MemoryObjectType.TILE);
        for (MemorySideCardEntry entry : tileEntries) {
            int x = entry.getX();
            int y = entry.getY();
            // 区域边界检查
            if (!isInAssignedZone(x, y, zone)) continue;

            // 检查是否已经被队友锁定
            if (isCellLockedByTeammate(x, y)) continue;

            // 检查记忆中该位置是否还有 Tile
            if (!isTileStillExists(x, y)) continue;

            int dist = manhattan(cx, cy, x, y);
            if (dist < bestDist) {
                bestDist = dist;
                // 从 memoryGrid 获取或创建
                TWEntity entity = (TWEntity) this.memory.getMemoryGrid().get(x, y);
                if (entity instanceof TWTile) {
                    best = (TWTile) entity;
                } else {
                    // 创建临时实体用于返回
                    double observedAt = entry.getLastSeen();
                    double death = observedAt + Parameters.lifeTime;
                    best = new TWTile(x, y, this.getEnvironment(), observedAt, death);
                }
            }
        }

        return best;
    }

    /**
     * 从本地感知 + 共同记忆中查找最近的 Hole（限制在分配区域内）
     */
    private TWHole findNearestHoleFromAllMemory() {
        TWHole best = null;
        int bestDist = Integer.MAX_VALUE;
        int cx = getX();
        int cy = getY();
        ZoneAssignment zone = hasZoneAssignment() ? getZoneAssignment() : null;

        // 1. 先检查本地感知范围内的 Hole（如果不在区域内则跳过）
        ObjectGrid2D grid = this.getEnvironment().getObjectGrid();
        int range = Parameters.defaultSensorRange;
        for (int dx = -range; dx <= range; dx++) {
            for (int dy = -range; dy <= range; dy++) {
                int x = cx + dx;
                int y = cy + dy;
                if (!this.getEnvironment().isInBounds(x, y)) continue;
                // 区域边界检查
                if (!isInAssignedZone(x, y, zone)) continue;

                TWEntity obj = (TWEntity) grid.get(x, y);
                if (obj instanceof TWHole) {
                    int dist = manhattan(cx, cy, x, y);
                    if (dist < bestDist) {
                        bestDist = dist;
                        best = (TWHole) obj;
                    }
                }
            }
        }

        // 2. 检查共同记忆中的 Hole (sideCard)（限制在区域内）
        List<MemorySideCardEntry> holeEntries = sideCard.listByType(MemoryObjectType.HOLE);
        for (MemorySideCardEntry entry : holeEntries) {
            int x = entry.getX();
            int y = entry.getY();
            // 区域边界检查
            if (!isInAssignedZone(x, y, zone)) continue;

            // 检查是否已经被队友锁定
            if (isCellLockedByTeammate(x, y)) continue;

            // 检查记忆中该位置是否还有 Hole
            if (!isHoleStillExists(x, y)) continue;

            int dist = manhattan(cx, cy, x, y);
            if (dist < bestDist) {
                bestDist = dist;
                // 从 memoryGrid 获取或创建
                TWEntity entity = (TWEntity) this.memory.getMemoryGrid().get(x, y);
                if (entity instanceof TWHole) {
                    best = (TWHole) entity;
                } else {
                    double observedAt = entry.getLastSeen();
                    double death = observedAt + Parameters.lifeTime;
                    best = new TWHole(x, y, this.getEnvironment(), observedAt, death);
                }
            }
        }

        return best;
    }

    /**
     * 检查 Tile 是否仍然存在
     */
    private boolean isTileStillExists(int x, int y) {
        // 1. 检查当前环境
        TWEntity envObj = (TWEntity) this.getEnvironment().getObjectGrid().get(x, y);
        if (envObj instanceof TWTile) {
            return true;
        }
        // 2. 检查共同记忆是否标记为非当前观察（可能是过时的）
        MemorySideCardEntry entry = sideCard.get(MemoryObjectType.TILE, x, y);
        if (entry != null && !entry.isSpawnObserved()) {
            return false;
        }
        return true;
    }

    /**
     * 检查 Hole 是否仍然存在
     */
    private boolean isHoleStillExists(int x, int y) {
        TWEntity envObj = (TWEntity) this.getEnvironment().getObjectGrid().get(x, y);
        if (envObj instanceof TWHole) {
            return true;
        }
        MemorySideCardEntry entry = sideCard.get(MemoryObjectType.HOLE, x, y);
        if (entry != null && !entry.isSpawnObserved()) {
            return false;
        }
        return true;
    }

    // ========== 目标有效性检测（每15步） ==========
    private boolean shouldCheckTargetValidity(long step) {
        if (!hasActiveTargetLock()) return false;
        if (lastValidatedTarget != null && !lastValidatedTarget.equals(lockedTargetCell)) {
            lastValidatedTarget = new Int2D(lockedTargetCell.x, lockedTargetCell.y);
            lastTargetCheckStep = step;
            return true;
        }
        return (step - lastTargetCheckStep) >= TARGET_CHECK_INTERVAL;
    }

    private boolean validateCurrentTarget(long step) {
        if (!hasActiveTargetLock()) return true;

        int tx = lockedTargetCell.x;
        int ty = lockedTargetCell.y;

        // 1. 检查是否被队友锁定
        TargetLease teammate = teammateTargetLeases.get(MemorySideCard.cellKey(tx, ty));
        if (teammate != null && !teammate.owner.equals(this.getName())) {
            if (!isPreferredLock(getName(), lockedTargetPriority, lockedTargetStep,
                    teammate.owner, teammate.priority, teammate.step)) {
                releaseOwnTargetLock("locked_by_teammate");
                return false;
            }
        }

        // 2. 检查目标是否仍然存在（基于本地感知或共同记忆）
        if (lockedTargetType == MemoryObjectType.TILE) {
            if (!isTileStillExists(tx, ty)) {
                releaseOwnTargetLock("tile_missing");
                return false;
            }
        } else if (lockedTargetType == MemoryObjectType.HOLE) {
            if (!isHoleStillExists(tx, ty)) {
                releaseOwnTargetLock("hole_missing");
                return false;
            }
        }

        lastTargetCheckStep = step;
        return true;
    }

    // ========== Path Planning with Caching (含贪心降级和重试机制) ==========
    private TWThought planToTarget(int tx, int ty) {
        if (getX() == tx && getY() == ty) {
            return waitThought();
        }

        // Check cache validity
        if (cachedPath != null && cachedTarget != null
                && cachedTarget.x == tx && cachedTarget.y == ty
                && !cachedPath.isEmpty()) {

            TWDirection nextDir = cachedPath.get(0);
            int nx = getX() + nextDir.dx;
            int ny = getY() + nextDir.dy;

            // Cache still valid if next step is not blocked
            if (!isBlocked(nx, ny)) {
                cachedPath.remove(0);
                return new TWThought(TWAction.MOVE, nextDir);
            }
            // Blocked, invalidate cache and replan
            clearCachedPath();
        }

        // Compute new path using A*
        List<TWDirection> newPath = limitedAStar(tx, ty);
        if (newPath != null && !newPath.isEmpty()) {
            cachedPath = newPath;
            cachedTarget = new Int2D(tx, ty);
            TWDirection nextDir = cachedPath.remove(0);
            return new TWThought(TWAction.MOVE, nextDir);
        }

        // A* 失败，尝试贪心降级
        TWThought greedyFallback = tryGreedyFallback(tx, ty);
        if (greedyFallback != null) {
            return greedyFallback;
        }

        // A* 和贪心都失败，记录重试次数
        return handlePathPlanningFailure(tx, ty);
    }

    /**
     * 处理路径规划失败的逻辑
     * 重试 MAX_PATH_PLANNING_RETRY 次后，释放锁并选择新目标
     */
    private TWThought handlePathPlanningFailure(int tx, int ty) {
        Int2D currentTarget = new Int2D(tx, ty);

        // 检查是否换了目标，换了目标则重置计数器
        if (lastRetryTarget == null || lastRetryTarget.x != tx || lastRetryTarget.y != ty) {
            pathPlanningRetryCount = 1;
            lastRetryTarget = currentTarget;
            return waitThought();
        }

        pathPlanningRetryCount++;

        if (pathPlanningRetryCount >= MAX_PATH_PLANNING_RETRY) {
            // 重试次数达到上限，释放锁并清除缓存
            releaseOwnTargetLock("path_unreachable");
            clearCachedPath();
            resetPathPlanningRetry();
            return waitThought();
        }

        return waitThought();
    }

    /**
     * 重置路径规划重试计数器
     */
    private void resetPathPlanningRetry() {
        pathPlanningRetryCount = 0;
        lastRetryTarget = null;
    }

    /**
     * 贪心降级: 向目标方向移动一步
     * 当 A* 找不到路径时，尝试向目标方向移动
     */
    private TWThought tryGreedyFallback(int tx, int ty) {
        int cx = getX();
        int cy = getY();
        int dx = tx - cx;
        int dy = ty - cy;

        // 尝试优先水平或垂直移动（曼哈顿距离更短的方向优先）
        TWDirection[] moveOrder;
        if (Math.abs(dx) >= Math.abs(dy)) {
            moveOrder = new TWDirection[]{
                dx > 0 ? TWDirection.E : (dx < 0 ? TWDirection.W : null),
                dy > 0 ? TWDirection.S : (dy < 0 ? TWDirection.N : null)
            };
        } else {
            moveOrder = new TWDirection[]{
                dy > 0 ? TWDirection.S : (dy < 0 ? TWDirection.N : null),
                dx > 0 ? TWDirection.E : (dx < 0 ? TWDirection.W : null)
            };
        }

        for (TWDirection dir : moveOrder) {
            if (dir == null) continue;
            int nx = cx + dir.dx;
            int ny = cy + dir.dy;
            // 检测静态障碍物和动态障碍物（其他智能体）
            if (isInBounds(nx, ny) && !isBlocked(nx, ny)) {
                clearCachedPath();  // 清除缓存，下次重新规划
                return new TWThought(TWAction.MOVE, dir);
            }
        }
        return null;
    }

    private void clearCachedPath() {
        cachedPath = null;
        cachedTarget = null;
    }

    private List<TWDirection> limitedAStar(int tx, int ty) {
        int x0 = getX(), y0 = getY();
        if (x0 == tx && y0 == ty) return null;

        Set<Long> closed = new HashSet<>();
        PriorityQueue<PathNode> open = new PriorityQueue<>(Comparator.comparingDouble(n -> n.f));
        open.add(new PathNode(x0, y0, null, 0, manhattan(x0, y0, tx, ty)));
        int count = 0;

        while (!open.isEmpty() && count < ASTAR_MAX_NODES) {
            count++;
            PathNode current = open.poll();
            if (current.x == tx && current.y == ty) {
                return reconstructPath(current);
            }
            closed.add(key(current.x, current.y));

            for (TWDirection d : TWDirection.values()) {
                if (d == TWDirection.Z) continue;
                int nx = current.x + d.dx;
                int ny = current.y + d.dy;
                if (!isInBounds(nx, ny) || isBlocked(nx, ny)) continue;
                if (closed.contains(key(nx, ny))) continue;
                if (Math.abs(nx - x0) > LOCAL_RANGE || Math.abs(ny - y0) > LOCAL_RANGE) continue;
                open.add(new PathNode(nx, ny, current, current.g + 1, manhattan(nx, ny, tx, ty)));
            }
        }
        return null;
    }

    private List<TWDirection> reconstructPath(PathNode node) {
        List<TWDirection> path = new ArrayList<>();
        while (node.parent != null) {
            TWDirection dir = directionFromParent(node);
            if (dir != null) path.add(0, dir);
            node = node.parent;
        }
        return path;
    }

    private TWDirection directionFromParent(PathNode node) {
        for (TWDirection d : TWDirection.values()) {
            if (d == TWDirection.Z) continue;
            if (node.parent.x + d.dx == node.x && node.parent.y + d.dy == node.y) {
                return d;
            }
        }
        return TWDirection.N;
    }

    // ========== 基于共同记忆的碰撞检测 ==========
    protected boolean isBlocked(int x, int y) {
        // 1. 检查边界
        if (!this.getEnvironment().isInBounds(x, y)) return true;
        // 2. 检查环境中的障碍物
        if (this.getEnvironment().isCellBlocked(x, y)) return true;
        // 3. 检查记忆中的障碍物（包括共同记忆）
        if (this.memory.isCellBlocked(x, y)) return true;
        // 4. 检查 sideCard 中的障碍物记录
        if (sideCard.get(MemoryObjectType.OBSTACLE, x, y) != null) return true;
        // 5. 检查是否有其他智能体占据该格子
        if (isCellOccupiedByOtherAgent(x, y)) return true;
        return false;
    }

    /**
     * 检测指定格子是否被其他智能体占据
     */
    private boolean isCellOccupiedByOtherAgent(int x, int y) {
        ObjectGrid2D agentGrid = this.getEnvironment().getAgentGrid();
        Object occupant = agentGrid.get(x, y);
        if (occupant instanceof TWAgent && !occupant.equals(this)) {
            return true;
        }
        return false;
    }

    private boolean isInBounds(int x, int y) {
        return this.getEnvironment().isInBounds(x, y);
    }

    private long key(int x, int y) {
        return ((long) x << 32) | (y & 0xFFFFFFFFL);
    }

    private int manhattan(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    private long currentStep() {
        return this.getEnvironment().schedule.getSteps();
    }

    private void publishActionEvent(CommType type, int x, int y, String payload) {
        publishProtocolMessage(type, x, y, payload);
    }

    // ========== Zone Boundary Helper ==========
    /**
     * 检查指定坐标是否在当前分配的区域内（带边界检查）
     */
    private boolean isInAssignedZone(int x, int y, ZoneAssignment zone) {
        if (zone == null) {
            return true;
        }
        return x >= zone.x1 && x <= zone.x2 && y >= zone.y1 && y <= zone.y2;
    }

    // ========== Internal Classes ==========
    private static class PathNode {
        int x, y;
        PathNode parent;
        int g;
        int f;

        PathNode(int x, int y, PathNode parent, int g, int h) {
            this.x = x;
            this.y = y;
            this.parent = parent;
            this.g = g;
            this.f = g + h;
        }
    }

    private static class TargetLease {
        final String owner;
        final double priority;
        final long step;
        final int ttlSteps;
        final long expiresAtStep;

        TargetLease(String owner, double priority, long step, int ttlSteps) {
            this.owner = owner;
            this.priority = priority;
            this.step = step;
            this.ttlSteps = Math.max(1, ttlSteps);
            this.expiresAtStep = this.step + this.ttlSteps;
        }
    }
}
