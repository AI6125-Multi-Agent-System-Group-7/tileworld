package tileworld.agent;

import java.util.*;
import java.util.stream.Collectors;

import sim.field.grid.ObjectGrid2D;
import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.agent.classdefines.MemoryObjectType;
import tileworld.agent.classdefines.MemorySideCard;
import tileworld.agent.classdefines.MemorySideCardEntry;
import tileworld.environment.*;
import tileworld.agent.utils.SensorSnapshotCodec;

/**
 * MyCustomAgent - A cooperative multi-agent with zone-based exploration
 * and target locking for Tileworld simulation.
 *
 * Key features:
 * - Zigzag exploration before zone assignment
 * - Zigzag scanning within assigned zone to find fuel station
 * - Cooperative path planning using A* with internal path nodes
 * - Target lock module for multi-agent coordination
 * - Message processing with MemorySideCard for shared memory
 */
public class AgentWu extends Group7AgentBase {

    // ===================== 配置常量 =====================
    private static final int FUEL_MARGIN = 20;
    private static final int LOCK_TTL_MIN_STEPS = 8;
    private static final int LOCK_TTL_MAX_STEPS = 20;
    private static final int LOCK_RENEW_BEFORE_STEPS = 2;
    private static final double LOCK_PRIORITY_EPSILON = 1e-9;
    private static final int ZONE_SWEEP_STRIDE = 7;

    // ===================== 状态变量 =====================
    private final MemorySideCard sideCard;
    private final Map<String, TargetLease> teammateTargetLeases;
    private final int ownTargetLockTtlSteps;

    // Owned target lock state
    private Int2D lockedTargetCell;
    private MemoryObjectType lockedTargetType;
    private double lockedTargetPriority;
    private long lockedTargetStep;
    private long lockedTargetExpiryStep;

    // Zone sweep state (for scanning zone to find fuel station)
    private String zoneSweepKey;
    private Int2D zoneSweepTarget;
    private int zoneSweepX;
    private int zoneSweepY;
    private boolean zoneSweepRight;
    private boolean zoneSweepDown;

    // Known fuel stations
    private final Set<Int2D> knownFuelStations;

    // Path planning state
    private List<CoopPathNode> currentCoopPath;
    private Int2D currentCoopTarget;

    // ===================== 构造方法 =====================
    public AgentWu(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
        this.sideCard = new MemorySideCard();
        this.teammateTargetLeases = new HashMap<>();
        this.ownTargetLockTtlSteps = computeDefaultTargetLockTtl(env);
        this.lockedTargetCell = null;
        this.lockedTargetType = null;
        this.lockedTargetPriority = 0.0;
        this.lockedTargetStep = -1L;
        this.lockedTargetExpiryStep = -1L;
        this.zoneSweepKey = null;
        this.zoneSweepTarget = null;
        this.zoneSweepX = 0;
        this.zoneSweepY = 0;
        this.zoneSweepRight = true;
        this.zoneSweepDown = true;
        this.knownFuelStations = new HashSet<>();
        this.currentCoopPath = null;
        this.currentCoopTarget = null;

        enableZoneCoordination();
    }

    // ===================== 必须重写的方法 =====================
    @Override
    public String getName() {
        return super.getName();
    }

    @Override
    protected TWThought think() {
        long step = currentStep();

        rememberFuelStationsInSensorRange();
        processIncomingMessages(step);
        pruneStaleTeammateTargetLeases(step);
        ensureActiveLockStillValid(step);

        // Zone coordination handled by base class processZoneCoordinationInThink()
        processZoneCoordinationInThink();

        // Before zone assignment: zigzag exploration to find fuel station
        if (!isZoneCoordinationComplete()) {
            if (!knownFuelStations.isEmpty()) {
                markZoneCoordinationComplete();
            } else {
                releaseOwnTargetLock("pre_zone_explore");
                return thinkPreZoneExplore();
            }
        }

        // After zone assignment but still searching for fuel station in zone
        if (knownFuelStations.isEmpty() && hasZoneAssignment()) {
            releaseOwnTargetLock("zone_scan");
            return thinkZoneScanFuelStation();
        }

        // At fuel station and need refuel
        if (this.getEnvironment().inFuelStation(this) && shouldRefuel(FUEL_MARGIN)) {
            releaseOwnTargetLock("refuel_now");
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }

        // Low fuel - go to station
        if (shouldRefuel(FUEL_MARGIN)) {
            releaseOwnTargetLock("refuel_trip");
            TWFuelStation station = findFuelStationInMemory();
            if (station != null) {
                return stepTowardCoopPath(new Int2D(station.getX(), station.getY()));
            }
            return exploreZigZag();
        }

        // Immediate actions
        TWEntity here = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
        if (here instanceof TWHole && this.hasTile()) {
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (here instanceof TWTile && this.carriedTiles.size() < 3) {
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        // Target picking logic
        return thinkPickTarget();
    }

    @Override
    protected void act(TWThought thought) {
        if (thought == null) {
            thought = waitThought();
        }

        try {
            switch (thought.getAction()) {
                case MOVE:
                    this.move(thought.getDirection());
                    if (currentCoopTarget != null &&
                        this.getX() == currentCoopTarget.x &&
                        this.getY() == currentCoopTarget.y) {
                        clearCoopPlan();
                    }
                    break;
                case PICKUP:
                    TWEntity tileObj = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                    if (tileObj instanceof TWTile && this.carriedTiles.size() < 3) {
                        this.pickUpTile((TWTile) tileObj);
                        sideCard.remove(MemoryObjectType.TILE, this.getX(), this.getY());
                    }
                    break;
                case PUTDOWN:
                    TWEntity holeObj = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                    if (holeObj instanceof TWHole && this.hasTile()) {
                        this.putTileInHole((TWHole) holeObj);
                        sideCard.remove(MemoryObjectType.HOLE, this.getX(), this.getY());
                    }
                    break;
                case REFUEL:
                    if (this.getEnvironment().inFuelStation(this)) {
                        this.refuel();
                    }
                    break;
            }
        } catch (Exception e) {
            clearCoopPlan();
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

    // ===================== 分区探索策略 =====================
    private TWThought thinkPreZoneExplore() {
        return exploreZigZag();
    }

    private TWThought thinkZoneScanFuelStation() {
        ZoneAssignment zone = getZoneAssignment();
        if (zone == null) {
            return exploreZigZag();
        }

        // Initialize sweep if needed
        resetZoneSweepIfNeeded(zone);

        for (int attempts = 0; attempts < 6; attempts++) {
            if (zoneSweepTarget == null || isAtPosition(zoneSweepTarget)) {
                zoneSweepTarget = nextZoneSweepTarget(zone);
            }

            TWThought thought = stepTowardCoopPath(zoneSweepTarget);
            if (!isWaitThought(thought)) {
                return thought;
            }

            zoneSweepTarget = nextZoneSweepTarget(zone);
            clearCoopPlan();
        }

        return waitThought();
    }

    private void resetZoneSweepIfNeeded(ZoneAssignment zone) {
        String key = zone.epoch + ":" + zone.x1 + ":" + zone.y1 + ":" + zone.x2 + ":" + zone.y2;
        if (key.equals(zoneSweepKey)) {
            return;
        }
        zoneSweepKey = key;
        zoneSweepTarget = null;
        zoneSweepX = zone.x1;
        zoneSweepY = zone.y1;
        zoneSweepRight = true;
        zoneSweepDown = true;
    }

    private Int2D nextZoneSweepTarget(ZoneAssignment zone) {
        if (zoneSweepRight) {
            int nextX = zoneSweepX + ZONE_SWEEP_STRIDE;
            if (nextX <= zone.x2) {
                zoneSweepX = nextX;
            } else {
                zoneSweepX = zone.x2;
                zoneSweepY = bounceY(zoneSweepY, zone.y1, zone.y2);
                zoneSweepRight = false;
            }
        } else {
            int nextX = zoneSweepX - ZONE_SWEEP_STRIDE;
            if (nextX >= zone.x1) {
                zoneSweepX = nextX;
            } else {
                zoneSweepX = zone.x1;
                zoneSweepY = bounceY(zoneSweepY, zone.y1, zone.y2);
                zoneSweepRight = true;
            }
        }

        if (zoneSweepY > zone.y2) {
            zoneSweepY = zone.y1;
        }

        return new Int2D(zoneSweepX, zoneSweepY);
    }

    private int bounceY(int currentY, int yMin, int yMax) {
        int signedStep = zoneSweepDown ? ZONE_SWEEP_STRIDE : -ZONE_SWEEP_STRIDE;
        int nextY = currentY + signedStep;
        if (nextY >= yMin && nextY <= yMax) {
            return nextY;
        }
        zoneSweepDown = !zoneSweepDown;
        signedStep = zoneSweepDown ? ZONE_SWEEP_STRIDE : -ZONE_SWEEP_STRIDE;
        nextY = currentY + signedStep;
        return Math.max(yMin, Math.min(yMax, nextY));
    }

    // ===================== 目标选择策略 =====================
    private TWThought thinkPickTarget() {
        long step = currentStep();

        // Try to pick up tile if not full
        if (carriedTiles.size() < 3) {
            TWTile nearestTile = (TWTile) memory.getClosestObjectInSensorRange(TWTile.class);
            if (nearestTile != null) {
                MemorySideCardEntry entry = sideCard.get(MemoryObjectType.TILE, nearestTile.getX(), nearestTile.getY());
                if (entry == null) {
                    entry = new MemorySideCardEntry(
                            MemoryObjectType.TILE,
                            nearestTile.getX(),
                            nearestTile.getY(),
                            step, step, false);
                    sideCard.upsert(MemoryObjectType.TILE, nearestTile.getX(), nearestTile.getY(), step, false);
                }
                if (tryAcquireOrKeepTargetLock(entry, step)) {
                    return stepTowardCoopPath(new Int2D(nearestTile.getX(), nearestTile.getY()));
                }
            }
        }

        // Try to fill hole if carrying tiles
        if (!carriedTiles.isEmpty()) {
            TWHole nearestHole = (TWHole) memory.getClosestObjectInSensorRange(TWHole.class);
            if (nearestHole != null) {
                MemorySideCardEntry entry = sideCard.get(MemoryObjectType.HOLE, nearestHole.getX(), nearestHole.getY());
                if (entry == null) {
                    entry = new MemorySideCardEntry(
                            MemoryObjectType.HOLE,
                            nearestHole.getX(),
                            nearestHole.getY(),
                            step, step, false);
                    sideCard.upsert(MemoryObjectType.HOLE, nearestHole.getX(), nearestHole.getY(), step, false);
                }
                if (tryAcquireOrKeepTargetLock(entry, step)) {
                    return stepTowardCoopPath(new Int2D(nearestHole.getX(), nearestHole.getY()));
                }
            }
        }

        // No valid target, explore
        releaseOwnTargetLock("no_target");
        return exploreZigZag();
    }

    // ===================== 协作战路径规划 (Solution 3) =====================
    private TWThought stepTowardCoopPath(Int2D target) {
        if (target == null) {
            return waitThought();
        }

        if (currentCoopTarget == null || !currentCoopTarget.equals(target) ||
            currentCoopPath == null || currentCoopPath.isEmpty()) {
            currentCoopTarget = target;
            currentCoopPath = planCoopPath(new Int2D(this.getX(), this.getY()), target);
        }

        if (currentCoopPath == null || currentCoopPath.isEmpty()) {
            return waitThought();
        }

        CoopPathNode next = currentCoopPath.remove(0);
        return new TWThought(TWAction.MOVE, next.direction);
    }

    private List<CoopPathNode> planCoopPath(Int2D start, Int2D goal) {
        if (start.equals(goal)) {
            return new ArrayList<>();
        }

        PriorityQueue<CoopPathNode> open = new PriorityQueue<>(
                Comparator.comparingInt(n -> n.fCost));
        Set<Int2D> closed = new HashSet<>();
        Map<Int2D, CoopPathNode> nodes = new HashMap<>();

        CoopPathNode startNode = new CoopPathNode(start, null, TWDirection.Z, 0, manhattan(start, goal));
        open.add(startNode);
        nodes.put(start, startNode);

        while (!open.isEmpty()) {
            CoopPathNode current = open.poll();
            if (current.pos.equals(goal)) {
                return reconstructCoopPath(current);
            }

            closed.add(current.pos);

            for (TWDirection dir : getCardinalDirections()) {
                Int2D next = new Int2D(current.pos.x + dir.dx, current.pos.y + dir.dy);
                if (!isInBounds(next.x, next.y)) {
                    continue;
                }
                if (closed.contains(next)) {
                    continue;
                }
                if (isCellBlockedForPath(next.x, next.y)) {
                    continue;
                }

                int g = current.gCost + 1;
                CoopPathNode existing = nodes.get(next);
                if (existing == null) {
                    CoopPathNode nextNode = new CoopPathNode(next, current, dir, g, manhattan(next, goal));
                    nodes.put(next, nextNode);
                    open.add(nextNode);
                } else if (g < existing.gCost) {
                    existing.gCost = g;
                    existing.fCost = g + existing.hCost;
                    existing.parent = current;
                    existing.direction = dir;
                    open.remove(existing);
                    open.add(existing);
                }
            }
        }

        return null;
    }

    private List<CoopPathNode> reconstructCoopPath(CoopPathNode node) {
        List<CoopPathNode> path = new ArrayList<>();
        CoopPathNode current = node;

        while (current.parent != null) {
            path.add(0, current);
            current = current.parent;
        }

        return path;
    }

    private boolean isCellBlockedForPath(int x, int y) {
        if (!isInBounds(x, y)) {
            return true;
        }
        if (this.getEnvironment().isCellBlocked(x, y)) {
            return true;
        }
        return this.memory.isCellBlocked(x, y);
    }

    private boolean isInBounds(int x, int y) {
        return x >= 0 && y >= 0 &&
               x < this.getEnvironment().getxDimension() &&
               y < this.getEnvironment().getyDimension();
    }

    private int manhattan(Int2D a, Int2D b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    private TWDirection[] getCardinalDirections() {
        return new TWDirection[]{TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W};
    }

    private void clearCoopPlan() {
        currentCoopPath = null;
        currentCoopTarget = null;
    }

    // ===================== 目标锁协作模块 =====================
    private static int computeDefaultTargetLockTtl(TWEnvironment env) {
        int span = Math.max(1, env.getxDimension() + env.getyDimension());
        int ttl = span / 6;
        return Math.max(LOCK_TTL_MIN_STEPS, Math.min(LOCK_TTL_MAX_STEPS, ttl));
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

    private boolean isLeasePreferred(TargetLease incoming, TargetLease existing) {
        if (Math.abs(incoming.priority - existing.priority) > LOCK_PRIORITY_EPSILON) {
            return incoming.priority > existing.priority;
        }
        if (incoming.step != existing.step) {
            return incoming.step < existing.step;
        }
        String safeA = (incoming.owner == null) ? "" : incoming.owner;
        String safeB = (existing.owner == null) ? "" : existing.owner;
        return safeA.compareTo(safeB) < 0;
    }

    private void pruneStaleTeammateTargetLeases(long step) {
        List<String> stale = new ArrayList<>();
        for (Map.Entry<String, TargetLease> entry : teammateTargetLeases.entrySet()) {
            if (step > entry.getValue().expiresAtStep) {
                stale.add(entry.getKey());
            }
        }
        for (String key : stale) {
            teammateTargetLeases.remove(key);
        }
    }

    private boolean hasActiveTargetLock() {
        return lockedTargetCell != null && lockedTargetType != null;
    }

    private boolean shouldRenewTargetLock(long step) {
        if (!hasActiveTargetLock()) return false;
        if (lockedTargetExpiryStep < 0) return true;
        return step >= (lockedTargetExpiryStep - LOCK_RENEW_BEFORE_STEPS);
    }

    private void ensureActiveLockStillValid(long step) {
        if (!hasActiveTargetLock()) return;

        if (step > lockedTargetExpiryStep) {
            releaseOwnTargetLock("local_lease_expired");
            return;
        }

        TargetLease teammate = teammateTargetLeases.get(
                MemorySideCard.cellKey(lockedTargetCell.x, lockedTargetCell.y));
        if (teammate != null) {
            if (!isPreferredLock(getName(), lockedTargetPriority, lockedTargetStep,
                    teammate.owner, teammate.priority, teammate.step)) {
                releaseOwnTargetLock("lost_priority");
            }
        }
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

    private boolean tryAcquireOrKeepTargetLock(MemorySideCardEntry entry, long step) {
        if (entry == null) return false;

        int x = entry.getX();
        int y = entry.getY();
        MemoryObjectType type = entry.getType();

        if (isOwnLockAt(type, x, y)) return true;

        double priority = computeLockPriority(type, x, y, step);
        if (isBlockedByPreferredTeammateLock(x, y, priority, step)) {
            return false;
        }

        releaseOwnTargetLock("switch_target");
        lockedTargetCell = new Int2D(x, y);
        lockedTargetType = type;
        lockedTargetPriority = priority;
        lockedTargetStep = step;
        lockedTargetExpiryStep = step + ownTargetLockTtlSteps;
        publishProtocolMessage(
                CommType.TARGET_LOCK, x, y,
                encodeTargetLockPayload(priority, ownTargetLockTtlSteps));
        return true;
    }

    private boolean isOwnLockAt(MemoryObjectType type, int x, int y) {
        return hasActiveTargetLock()
                && lockedTargetType == type
                && lockedTargetCell.x == x
                && lockedTargetCell.y == y;
    }

    private boolean isBlockedByPreferredTeammateLock(int x, int y, double myPriority, long myStep) {
        TargetLease teammate = teammateTargetLeases.get(MemorySideCard.cellKey(x, y));
        if (teammate == null) return false;
        return !isPreferredLock(getName(), myPriority, myStep,
                teammate.owner, teammate.priority, teammate.step);
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

        long positive = seed & Long.MAX_VALUE;
        return (positive % 1000000L) / 1000000.0;
    }

    private void releaseOwnTargetLock(String reason) {
        if (!hasActiveTargetLock()) return;

        publishProtocolMessage(
                CommType.TARGET_RELEASE,
                lockedTargetCell.x, lockedTargetCell.y,
                (reason == null) ? "" : reason);

        lockedTargetCell = null;
        lockedTargetType = null;
        lockedTargetPriority = 0.0;
        lockedTargetStep = -1L;
        lockedTargetExpiryStep = -1L;
    }

    // ===================== 消息处理 =====================
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
                    sideCard.upsert(MemoryObjectType.TILE, parsed.x, parsed.y, parsed.step, false);
                    break;
                case OBS_NEW_HOLE:
                    sideCard.upsert(MemoryObjectType.HOLE, parsed.x, parsed.y, parsed.step, false);
                    break;
                case OBS_OBSTACLE:
                    sideCard.upsert(MemoryObjectType.OBSTACLE, parsed.x, parsed.y, parsed.step, false);
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

    private void processSnapshotMessage(ParsedMessage parsed) {
        List<SensorSnapshotCodec.SnapshotItem> items = SensorSnapshotCodec.decode(parsed.payload);
        for (SensorSnapshotCodec.SnapshotItem item : items) {
            if ("E".equals(item.code)) {
                continue;
            }

            MemoryObjectType type = MemoryObjectType.fromSnapshotCode(item.code);
            if (type == null) continue;

            sideCard.upsert(type, item.x, item.y, parsed.step, false);
        }
    }

    private void rememberFuelStationFromMessage(int x, int y) {
        knownFuelStations.add(new Int2D(x, y));
        ObjectGrid2D memoryGrid = this.memory.getMemoryGrid();
        if (!(memoryGrid.get(x, y) instanceof TWFuelStation)) {
            memoryGrid.set(x, y, new TWFuelStation(x, y, this.getEnvironment()));
        }
    }

    // ===================== 内部路径节点类 =====================
    private static class CoopPathNode {
        final Int2D pos;
        CoopPathNode parent;
        TWDirection direction;
        int gCost;
        final int hCost;
        int fCost;

        CoopPathNode(Int2D pos, CoopPathNode parent, TWDirection direction, int gCost, int hCost) {
            this.pos = pos;
            this.parent = parent;
            this.direction = direction;
            this.gCost = gCost;
            this.hCost = hCost;
            this.fCost = gCost + hCost;
        }
    }

    // ===================== 辅助方法 =====================
    private long currentStep() {
        return this.getEnvironment().schedule.getSteps();
    }

    private boolean isAtPosition(Int2D target) {
        return target != null && this.getX() == target.x && this.getY() == target.y;
    }

    private boolean isWaitThought(TWThought thought) {
        return thought != null
                && thought.getAction() == TWAction.MOVE
                && thought.getDirection() == TWDirection.Z;
    }
}
