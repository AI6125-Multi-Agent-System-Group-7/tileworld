package tileworld.agent;

import java.util.*;

import sim.field.grid.ObjectGrid2D;
import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.agent.utils.SensorSnapshotCodec;
import tileworld.agent.utils.SensorSnapshotCodec.SnapshotItem;
import tileworld.environment.*;

/**
 * AgentEnge - 智能Tileworld智能体实现
 *
 * 继承 {@link Group7AgentBase} 以启用 G7P2（SS/NT/NH/OB/FS/PK/FH、ZA/ZK 分区），
 * 便于与组内其他 agent 协作；决策逻辑仍为本地 A* + 优先级状态机。
 *
 * 实现优先级决策逻辑：
 * 1. 求生模式（fuel < 150）
 * 2. 填洞模式（有Tile且知道Hole位置）
 * 3. 拾取模式（空间未满且有Tile）
 * 4. 探索模式（犁地式扫描）
 */
public class AgentEnge extends Group7AgentBase {

    /** 与 G7P2 TL/TR 对齐：目标格是 Tile 还是 Hole */
    private enum LockKind {
        TILE,
        HOLE
    }

    private static final int LOCK_TTL_MIN_STEPS = 8;
    private static final int LOCK_TTL_MAX_STEPS = 20;
    private static final int LOCK_RENEW_BEFORE_STEPS = 2;
    private static final double LOCK_PRIORITY_EPSILON = 1e-9;

    /** 分区 bootstrap：地图越大允许越久；超时仍在区内则结束，避免永远不进入主策略 */
    private static final int ZONE_BOOTSTRAP_TIMEOUT_FACTOR = 4;

    // 犁地式扫描相关状态
    private int scanRow = 3; // 从第四行开始（y=3），传感器范围3可以覆盖y=0到y=6（7个单位）
    private int scanCol = 3; // 从第四列开始（x=3），传感器范围3可以覆盖x=0到x=6（7个单位）
    private boolean scanDirectionRight = true; // true = 向右, false = 向左
    private static final int SCAN_STEP = 7; // 犁地式扫描步长
    
    // 重规划相关
    private int replanAttempts = 0; // 重规划尝试次数
    private static final int MAX_REPLAN_ATTEMPTS = 3; // 最大重规划次数
    
    // 分区启动阶段：与 AgentHanny 一致的区内扫地锚点（由 Manager 的 ZA 驱动）
    private String zoneSweepKey;
    private Int2D zoneSweepTarget;
    private int zoneSweepX;
    private int zoneSweepY;
    private boolean zoneSweepRight;

    // G7P2 目标锁（TL/TR），与 AgentHanny 仲裁规则一致
    private final Map<String, TargetLease> teammateTargetLeases;
    private final int ownTargetLockTtlSteps;
    private Int2D lockedTargetCell;
    private LockKind lockedTargetKind;
    private double lockedTargetPriority;
    private long lockedTargetStep;
    private long lockedTargetExpiryStep;

    /** 来自队友 SS 报文头中的 (x,y)，用于探索阶段软排斥、减少传感器叠在一起 */
    private final Map<String, Int2D> teammatePositionsFromSnapshot;

    /** 上一次移动前的位置：用于避免“两格互跳” */
    private Int2D lastMoveFrom;

    /** 最近访问过的位置（短历史），用于去抖/防循环 */
    private final Deque<Int2D> recentPositions;

    /** 更长历史（探索阶段防循环层） */
    private final Deque<Int2D> positionHistory;

    /** 连续原地等待次数（MOVE(Z)） */
    private int consecutiveWaitMoves = 0;

    // 目标对象（用于追踪）
    private TWEntity targetEntity = null;

    public AgentEnge(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
        this.zoneSweepKey = null;
        this.zoneSweepTarget = null;
        this.zoneSweepX = 0;
        this.zoneSweepY = 0;
        this.zoneSweepRight = true;
        this.teammateTargetLeases = new HashMap<String, TargetLease>();
        this.teammatePositionsFromSnapshot = new HashMap<String, Int2D>();
        this.lastMoveFrom = null;
        this.recentPositions = new ArrayDeque<Int2D>();
        this.positionHistory = new ArrayDeque<Int2D>();
        this.ownTargetLockTtlSteps = computeDefaultTargetLockTtl(env);
        this.lockedTargetCell = null;
        this.lockedTargetKind = null;
        this.lockedTargetPriority = 0.0;
        this.lockedTargetStep = -1L;
        this.lockedTargetExpiryStep = -1L;
        enableZoneCoordination();
        applyExploreScanPhaseOffsetFromName(name, env);
    }

    @Override
    protected TWThought think() {
        processIncomingCooperativeMessages();
        rememberFuelStationsInSensorRange();
        processZoneCoordinationInThink();

        long step = getEnvironment().schedule.getSteps();
        if (!isZoneCoordinationComplete()) {
            // 低油优先：不必等分区阶段走完再走求生
            if (this.getFuelLevel() < 180) {
                markZoneCoordinationComplete();
            } else {
                ZoneAssignment zone = getZoneAssignment();
                boolean inAssignedZone = zone != null && zone.contains(this.getX(), this.getY());
                boolean knowsFuel = findFuelStationInMemory() != null;
                int maxBootstrapSteps = ZONE_BOOTSTRAP_TIMEOUT_FACTOR
                        * (getEnvironment().getxDimension() + getEnvironment().getyDimension());
                boolean timedOutInZone = inAssignedZone && step > maxBootstrapSteps;

                // 策略（与小组约定一致）：
                // - 在「尚无任何一方把油站写入全队认知」之前：保持 bootstrap，Manager 发 ZA，各 agent 去分配区域并在区内扫（基类每步会尝试找油站写记忆并广播 FS）。
                // - 一旦有 agent 在路上或区内「感知到」油站 → 本地记忆有油站、且会通过 FS 让队友记忆更新 → knowsFuel 为真，全员可结束分区阶段，改做 tile/hole。
                // - 区内久无油站则超时退出，避免卡住。
                if (knowsFuel || timedOutInZone) {
                    markZoneCoordinationComplete();
                } else {
                    clearPlan();
                    releaseOwnTargetLock("zone_bootstrap");
                    return bootstrapZoneSearch(step);
                }
            }
        }

        pruneStaleTeammateTargetLeases(step);
        ensureActiveLockStillValid(step);

        // 检查视野内的对象并记录（优先处理FuelStation）
        logVisibleObjects();
        
        // 优先级1: 求生模式 - fuel < 180（停止所有其他任务）
        if (this.getFuelLevel() < 180) {
            System.out.println(String.format("[%s] [模式: 求生] Fuel=%d < 180, 进入求生模式，停止所有其他任务", 
                getName(), (int)this.getFuelLevel()));
            // 清除所有其他任务的路径和目标
            currentPath = null;
            currentTarget = null;
            releaseOwnTargetLock("survival_mode");
            return thinkSurvivalMode();
        }
        
        // 优先级2: 填洞模式 - 有Tile且记忆中有Hole
        if (this.hasTile() && hasKnownHole()) {
            System.out.println(String.format("[%s] [模式: 填洞] 携带Tile数=%d, 已知Hole存在", 
                getName(), this.carriedTiles.size()));
            return thinkFillHoleMode(step);
        }
        
        // 优先级3: 拾取模式 - 空间未满且视野内有Tile
        if (this.carriedTiles.size() < 3) {
            TWTile visibleTile = (TWTile) this.memory.getClosestObjectInSensorRange(TWTile.class);
            if (visibleTile != null) {
                System.out.println(String.format("[%s] [模式: 拾取] 空间未满(当前=%d/3), 视野内有Tile在(%d,%d)", 
                    getName(), this.carriedTiles.size(), visibleTile.getX(), visibleTile.getY()));
                return thinkPickupMode(visibleTile, step);
            }
        }
        
        // 优先级4: 探索模式 - 犁地式扫描
        System.out.println(String.format("[%s] [模式: 探索] 位置=(%d,%d), 扫描目标=(%d,%d)", 
            getName(), this.getX(), this.getY(), scanCol, scanRow));
        return thinkExplorationMode();
    }
    
    /**
     * 记录视野内可见的对象，并手动将FuelStation存入记忆
     */
    private void logVisibleObjects() {
        // 优先检查FuelStation（因为它是TWEntity不是TWObject，不会被自动存入记忆）
        int sensorRange = Parameters.defaultSensorRange;
        sim.field.grid.ObjectGrid2D objectGrid = this.getEnvironment().getObjectGrid();
        
        // 扫描传感器范围内的所有对象，查找FuelStation
        for (int dx = -sensorRange; dx <= sensorRange; dx++) {
            for (int dy = -sensorRange; dy <= sensorRange; dy++) {
                int x = this.getX() + dx;
                int y = this.getY() + dy;
                
                if (this.getEnvironment().isInBounds(x, y)) {
                    Object obj = objectGrid.get(x, y);
                    if (obj instanceof TWFuelStation) {
                        TWFuelStation fuelStation = (TWFuelStation) obj;
                        // 手动存入记忆（因为FuelStation不是TWObject，不会被自动存入）
                        sim.field.grid.ObjectGrid2D memoryGrid = this.memory.getMemoryGrid();
                        Object memObj = memoryGrid.get(x, y);
                        if (!(memObj instanceof TWFuelStation)) {
                            // 存入记忆
                            memoryGrid.set(x, y, fuelStation);
                            System.out.println(String.format("[%s] [感知] 发现FuelStation在(%d,%d), 距离=%.1f [已手动标记到记忆]", 
                                getName(), x, y, this.getDistanceTo(fuelStation)));
                        }
                    }
                }
            }
        }
        
        // 检查视野内的其他对象（传感器范围内）
        TWTile tile = (TWTile) this.memory.getClosestObjectInSensorRange(TWTile.class);
        TWHole hole = (TWHole) this.memory.getClosestObjectInSensorRange(TWHole.class);
        
        if (tile != null) {
            System.out.println(String.format("[%s] [感知] 看到Tile在(%d,%d), 距离=%.1f [已标记到记忆]", 
                getName(), tile.getX(), tile.getY(), this.getDistanceTo(tile)));
        }
        if (hole != null) {
            System.out.println(String.format("[%s] [感知] 看到Hole在(%d,%d), 距离=%.1f [已标记到记忆]", 
                getName(), hole.getX(), hole.getY(), this.getDistanceTo(hole)));
        }
    }
    
    /**
     * 求生模式：寻找燃料站（停止所有其他任务）
     */
    private TWThought thinkSurvivalMode() {
        // 检查是否已经在燃料站
        if (this.getEnvironment().inFuelStation(this)) {
            System.out.println(String.format("[%s] [求生模式] 已在燃料站位置，执行加油", getName()));
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }
        
        // 尝试从记忆中查找燃料站
        TWFuelStation fuelStation = findFuelStationInMemory();
        
        if (fuelStation != null) {
            System.out.println(String.format("[%s] [求生模式] 从记忆中找到FuelStation在(%d,%d)，规划A*路径", 
                getName(), fuelStation.getX(), fuelStation.getY()));
            // 已知燃料站位置，使用A*规划路径（带重规划）
            Int2D target = new Int2D(fuelStation.getX(), fuelStation.getY());
            List<TWDirection> path = aStarPathfindingWithReplan(new Int2D(this.getX(), this.getY()), target, fuelStation);
            
            if (path != null && !path.isEmpty()) {
                this.currentPath = path;
                this.currentTarget = target;
                System.out.println(String.format("[%s] [求生模式] A*路径规划成功，路径长度=%d，下一步方向=%s", 
                    getName(), path.size(), path.get(0)));
                return new TWThought(TWAction.MOVE, path.get(0));
            } else {
                System.out.println(String.format("[%s] [求生模式] A*路径规划失败，使用感知找洞模式寻找FuelStation", getName()));
                // A*失败，使用感知找洞模式
                return thinkSurvivalExplorationMode();
            }
        } else {
            System.out.println(String.format("[%s] [求生模式] 记忆中未找到FuelStation，使用感知找洞模式寻找", getName()));
            // 未知燃料站位置，使用感知找洞模式寻找
            return thinkSurvivalExplorationMode();
        }
    }
    
    /**
     * 求生模式的探索：优先寻找FuelStation
     */
    private TWThought thinkSurvivalExplorationMode() {
        // 如果当前有路径且目标位置有效，继续执行
        if (currentPath != null && !currentPath.isEmpty() && currentTarget != null) {
            // 检查是否已经到达目标
            if (this.getX() == currentTarget.x && this.getY() == currentTarget.y) {
                currentPath = null;
                currentTarget = null;
            } else {
                TWDirection nextDir = currentPath.remove(0);
                System.out.println(String.format("[%s] [求生探索] 继续执行路径，剩余步数=%d，下一步=%s", 
                    getName(), currentPath.size(), nextDir));
                return new TWThought(TWAction.MOVE, nextDir);
            }
        }
        
        // 计算下一个扫描目标位置
        Int2D nextScanPos = calculateNextScanPosition();
        
        if (nextScanPos != null) {
            // 限制探索模式最多移动到y=46（除非y=47-49有FuelStation需要寻找）
            int maxY = 46;
            if (nextScanPos.y > maxY) {
                // 检查y=47-49是否有FuelStation
                boolean hasFuelStationInRightArea = false;
                for (int y = 47; y <= 49; y++) {
                    if (y < this.getEnvironment().getyDimension()) {
                        sim.field.grid.ObjectGrid2D memoryGrid = this.memory.getMemoryGrid();
                        Object obj = memoryGrid.get(this.getX(), y);
                        if (obj instanceof TWFuelStation) {
                            hasFuelStationInRightArea = true;
                            break;
                        }
                    }
                }
                
                if (!hasFuelStationInRightArea) {
                    // 没有FuelStation在y=47-49，限制扫描位置到y=46
                    nextScanPos = new Int2D(nextScanPos.x, maxY);
                    System.out.println(String.format("[%s] [求生探索] 限制扫描位置到y=%d（y=47-49无FuelStation）", 
                        getName(), maxY));
                }
            }
            
            System.out.println(String.format("[%s] [求生探索] 计算扫描目标=(%d,%d)，当前位置=(%d,%d)", 
                getName(), nextScanPos.x, nextScanPos.y, this.getX(), this.getY()));
            // 规划路径到扫描位置
            List<TWDirection> path = aStarPathfinding(new Int2D(this.getX(), this.getY()), nextScanPos);
            
            if (path != null && !path.isEmpty()) {
                this.currentPath = new ArrayList<>(path);
                this.currentTarget = nextScanPos;
                System.out.println(String.format("[%s] [求生探索] A*路径规划成功，路径长度=%d，下一步=%s", 
                    getName(), path.size(), path.get(0)));
                return new TWThought(TWAction.MOVE, path.get(0));
            } else {
                System.out.println(String.format("[%s] [求生探索] A*路径规划失败，无法到达扫描目标，使用随机方向", getName()));
            }
        }
        
        // 如果无法规划路径，使用简单的方向移动（也要限制y坐标）
        TWThought thought = getRandomValidDirection();
        // 如果随机方向会导致y>46，检查是否有FuelStation在y=47-49
        int nextY = this.getY();
        if (thought.getDirection() == TWDirection.S) {
            nextY = this.getY() + 1;
        }
        
        if (nextY > 46) {
            // 检查y=47-49是否有FuelStation
            boolean hasFuelStationInRightArea = false;
            for (int y = 47; y <= 49; y++) {
                if (y < this.getEnvironment().getyDimension()) {
                    sim.field.grid.ObjectGrid2D memoryGrid = this.memory.getMemoryGrid();
                    Object obj = memoryGrid.get(this.getX(), y);
                    if (obj instanceof TWFuelStation) {
                        hasFuelStationInRightArea = true;
                        break;
                    }
                }
            }
            
            if (!hasFuelStationInRightArea && thought.getDirection() == TWDirection.S) {
                // 没有FuelStation，限制移动方向（避免往南移动超过y=46）
                System.out.println(String.format("[%s] [求生探索] 限制移动方向，避免超过y=46", getName()));
                // 尝试其他方向
                TWDirection[] alternatives = {TWDirection.N, TWDirection.E, TWDirection.W};
                for (TWDirection altDir : alternatives) {
                    int testX = this.getX() + altDir.dx;
                    int testY = this.getY() + altDir.dy;
                    if (this.getEnvironment().isInBounds(testX, testY) && 
                        !this.getEnvironment().isCellBlocked(testX, testY) &&
                        !this.memory.isCellBlocked(testX, testY) &&
                        testY <= 46) {
                        thought = new TWThought(TWAction.MOVE, altDir);
                        break;
                    }
                }
            }
        }
        
        System.out.println(String.format("[%s] [求生探索] 使用随机方向移动，方向=%s", 
            getName(), thought.getDirection()));
        return thought;
    }
    
    /**
     * 填洞模式：带着Tile去填洞
     */
    private TWThought thinkFillHoleMode(long step) {
        TWHole nearestHole = selectHoleForFill(step);
        
        if (nearestHole != null) {
            System.out.println(String.format("[%s] [填洞模式] 找到最近Hole在(%d,%d)，当前位置=(%d,%d)，距离=%.1f", 
                getName(), nearestHole.getX(), nearestHole.getY(), 
                this.getX(), this.getY(), this.getDistanceTo(nearestHole)));
            
            // 检查是否已经在Hole位置
            if (this.getX() == nearestHole.getX() && this.getY() == nearestHole.getY()) {
                // 验证当前位置是否真的有Hole（因为对象可能会消失）
                Object objAtPos = this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                if (objAtPos instanceof TWHole) {
                    System.out.println(String.format("[%s] [填洞模式] 已在Hole位置，执行DROP", getName()));
                    return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
                } else {
                    // Hole已经消失，从记忆中移除
                    System.out.println(String.format("[%s] [填洞模式] Hole在(%d,%d)已消失，从记忆中移除", 
                        getName(), nearestHole.getX(), nearestHole.getY()));
                    this.memory.removeObject(nearestHole);
                    releaseOwnTargetLock("hole_gone_at_cell");
                    // 清除当前路径和目标
                    currentPath = null;
                    currentTarget = null;
                    // 查找下一个Hole
                    return findNextHoleAndPlan(step);
                }
            }
            
            // 验证目标Hole是否仍然存在（在规划路径前）
            Object objAtTarget = this.getEnvironment().getObjectGrid().get(nearestHole.getX(), nearestHole.getY());
            if (!(objAtTarget instanceof TWHole)) {
                // Hole已经消失，从记忆中移除
                System.out.println(String.format("[%s] [填洞模式] 目标Hole在(%d,%d)已消失，从记忆中移除", 
                    getName(), nearestHole.getX(), nearestHole.getY()));
                this.memory.removeObject(nearestHole);
                releaseOwnTargetLock("hole_gone_planned");
                // 清除当前路径和目标
                currentPath = null;
                currentTarget = null;
                // 查找下一个Hole
                return findNextHoleAndPlan(step);
            }
            
            // 在移动过程中，如果视野内有Tile且空间未满，先拾取
            TWTile visibleTile = (TWTile) this.memory.getClosestObjectInSensorRange(TWTile.class);
            if (visibleTile != null && this.carriedTiles.size() < 3) {
                // 如果Tile就在当前位置，先拾取
                if (this.getX() == visibleTile.getX() && this.getY() == visibleTile.getY()) {
                    System.out.println(String.format("[%s] [填洞模式] 当前位置有Tile，先拾取", getName()));
                    return new TWThought(TWAction.PICKUP, TWDirection.Z);
                }
            }
            
            // 检查是否有未完成的路径
            if (currentPath != null && !currentPath.isEmpty() && currentTarget != null) {
                if (currentTarget.x == nearestHole.getX() && currentTarget.y == nearestHole.getY()) {
                    // 继续执行已有路径
                    TWDirection nextDir = currentPath.remove(0);
                    System.out.println(String.format("[%s] [填洞模式] 继续执行已有路径，剩余步数=%d，下一步=%s", 
                        getName(), currentPath.size(), nextDir));
                    return new TWThought(TWAction.MOVE, nextDir);
                } else {
                    // 目标已改变，重新规划
                    System.out.println(String.format("[%s] [填洞模式] 目标已改变，清除旧路径", getName()));
                    currentPath = null;
                    currentTarget = null;
                }
            }
            
            // 规划去Hole的路径（带重规划）
            Int2D target = new Int2D(nearestHole.getX(), nearestHole.getY());
            System.out.println(String.format("[%s] [填洞模式] 规划A*路径到Hole(%d,%d)", 
                getName(), target.x, target.y));
            List<TWDirection> path = aStarPathfindingWithReplan(new Int2D(this.getX(), this.getY()), target, nearestHole);
            
            if (path != null && !path.isEmpty()) {
                this.currentPath = path;
                this.currentTarget = target;
                System.out.println(String.format("[%s] [填洞模式] A*路径规划成功，路径长度=%d，下一步=%s", 
                    getName(), path.size(), path.get(0)));
                return new TWThought(TWAction.MOVE, path.get(0));
            } else {
                System.out.println(String.format("[%s] [填洞模式] A*路径规划失败，无法到达Hole", getName()));
                releaseOwnTargetLock("fill_unreachable");
            }
        } else {
            System.out.println(String.format("[%s] [填洞模式] 记忆中未找到可锁定Hole，切换到探索模式", getName()));
        }
        
        // 如果找不到Hole，回到探索模式
        return thinkExplorationMode();
    }
    
    /**
     * 查找下一个Hole并规划路径
     */
    private TWThought findNextHoleAndPlan(long step) {
        TWHole nextHole = selectHoleForFill(step);

        if (nextHole != null) {
            System.out.println(String.format("[%s] [填洞模式] 找到下一个Hole在(%d,%d)，距离=%.1f", 
                getName(), nextHole.getX(), nextHole.getY(), this.getDistanceTo(nextHole)));
            
            // 验证这个Hole是否仍然存在
            Object objAtTarget = this.getEnvironment().getObjectGrid().get(nextHole.getX(), nextHole.getY());
            if (!(objAtTarget instanceof TWHole)) {
                // 这个Hole也消失了，递归查找下一个
                System.out.println(String.format("[%s] [填洞模式] Hole在(%d,%d)也已消失，继续查找", 
                    getName(), nextHole.getX(), nextHole.getY()));
                this.memory.removeObject(nextHole);
                releaseOwnTargetLock("next_hole_gone");
                return findNextHoleAndPlan(step);
            }
            
            // 规划路径（带重规划）
            Int2D target = new Int2D(nextHole.getX(), nextHole.getY());
            List<TWDirection> path = aStarPathfindingWithReplan(new Int2D(this.getX(), this.getY()), target, nextHole);
            
            if (path != null && !path.isEmpty()) {
                this.currentPath = path;
                this.currentTarget = target;
                System.out.println(String.format("[%s] [填洞模式] 规划到下一个Hole的路径成功，路径长度=%d，下一步=%s", 
                    getName(), path.size(), path.get(0)));
                return new TWThought(TWAction.MOVE, path.get(0));
            }
            releaseOwnTargetLock("next_fill_unreachable");
        }
        
        // 如果记忆中没有其他Hole了，切换到探索模式
        System.out.println(String.format("[%s] [填洞模式] 记忆中没有其他Hole，切换到探索模式寻找新Hole", getName()));
        return thinkExplorationMode();
    }
    
    /**
     * 拾取模式：拾取Tile
     */
    private TWThought thinkPickupMode(TWTile tile, long step) {
        if (!tryAcquireOrKeepLock(LockKind.TILE, tile.getX(), tile.getY(), step)) {
            System.out.println(String.format("[%s] [拾取模式] 格(%d,%d)被队友高优先级锁占用，放弃", 
                getName(), tile.getX(), tile.getY()));
            return thinkExplorationMode();
        }
        // 检查是否已经在Tile位置
        if (this.getX() == tile.getX() && this.getY() == tile.getY()) {
            System.out.println(String.format("[%s] [拾取模式] 已在Tile位置，执行PICKUP", getName()));
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }
        
        // 检查是否有未完成的路径
        if (currentPath != null && !currentPath.isEmpty() && currentTarget != null) {
            if (currentTarget.x == tile.getX() && currentTarget.y == tile.getY()) {
                // 继续执行已有路径
                TWDirection nextDir = currentPath.remove(0);
                System.out.println(String.format("[%s] [拾取模式] 继续执行已有路径，剩余步数=%d，下一步=%s", 
                    getName(), currentPath.size(), nextDir));
                return new TWThought(TWAction.MOVE, nextDir);
            }
        }
        
        // 规划去Tile的路径（带重规划）
        Int2D target = new Int2D(tile.getX(), tile.getY());
        System.out.println(String.format("[%s] [拾取模式] 规划A*路径到Tile(%d,%d)", 
            getName(), target.x, target.y));
        List<TWDirection> path = aStarPathfindingWithReplan(new Int2D(this.getX(), this.getY()), target, tile);
        
        if (path != null && !path.isEmpty()) {
            this.currentPath = path;
            this.currentTarget = target;
            System.out.println(String.format("[%s] [拾取模式] A*路径规划成功，路径长度=%d，下一步=%s", 
                getName(), path.size(), path.get(0)));
            return new TWThought(TWAction.MOVE, path.get(0));
        } else {
            System.out.println(String.format("[%s] [拾取模式] A*路径规划失败，无法到达Tile，切换到探索模式", getName()));
            releaseOwnTargetLock("pickup_unreachable");
        }
        
        // 如果无法到达，回到探索模式
        return thinkExplorationMode();
    }
    
    /**
     * 探索模式：犁地式扫描
     */
    private TWThought thinkExplorationMode() {
        releaseOwnTargetLock("explore");

        // -------- 探索模式强制防循环 --------
        if ((currentPath == null || currentPath.isEmpty()) && detectExplorationOscillation()) {
            currentPath = null;
            currentTarget = null;
            advanceScanCursor(); // 推进犁地游标，避免一直尝试同一个点
            // 不要无条件等待，否则 positionHistory 可能一直满足“原地卡住”
            return getAnyValidDirection();
        }

        // 如果当前有路径且目标位置有效，继续执行
        if (currentPath != null && !currentPath.isEmpty() && currentTarget != null) {
            // 检查是否已经到达目标
            if (this.getX() == currentTarget.x && this.getY() == currentTarget.y) {
                System.out.println(String.format("[%s] [探索模式] 已到达扫描目标(%d,%d)，清除路径", 
                    getName(), currentTarget.x, currentTarget.y));
                currentPath = null;
                currentTarget = null;
            } else {
                TWDirection nextDir = currentPath.remove(0);
                System.out.println(String.format("[%s] [探索模式] 继续执行路径，剩余步数=%d，下一步=%s，目标=(%d,%d)", 
                    getName(), currentPath.size(), nextDir, currentTarget.x, currentTarget.y));
                // 禁止立刻走回上一格，减少两格互跳
                if (lastMoveFrom != null
                        && lastMoveFrom.x == this.getX() + nextDir.dx
                        && lastMoveFrom.y == this.getY() + nextDir.dy) {
                    currentPath = null;
                    currentTarget = null;
                    advanceScanCursor();
                    return getAnyValidDirection();
                }
                return new TWThought(TWAction.MOVE, nextDir);
            }
        }
        
        // 计算下一个扫描目标位置
        Int2D nextScanPos = calculateNextScanPosition();
        
        if (nextScanPos != null) {
            if (isZoneCoordinationComplete() && hasZoneAssignment()) {
                nextScanPos = clipPointToAssignedZone(nextScanPos);
            }
            nextScanPos = nudgeExploreAwayFromTeammates(nextScanPos);

            // 已在扫描目标点：推进游标，避免 path.isEmpty() 被当成失败导致随机走动
            if (this.getX() == nextScanPos.x && this.getY() == nextScanPos.y) {
                advanceScanCursor();
                return getAnyValidDirection();
            }

            // 限制探索模式最多移动到y=46（除非y=47-49有物体需要拾取）
            int maxY = 46;
            if (nextScanPos.y > maxY) {
                // 检查y=47-49是否有Tile或Hole需要拾取
                boolean hasTargetInRightArea = false;
                for (int y = 47; y <= 49; y++) {
                    if (y < this.getEnvironment().getyDimension()) {
                        TWTile tile = this.memory.getNearbyTile(this.getX(), y, Double.MAX_VALUE);
                        TWHole hole = this.memory.getNearbyHole(this.getX(), y, Double.MAX_VALUE);
                        if (tile != null || hole != null) {
                            hasTargetInRightArea = true;
                            break;
                        }
                    }
                }
                
                if (!hasTargetInRightArea) {
                    // 没有目标在y=47-49，限制扫描位置到y=46
                    nextScanPos = new Int2D(nextScanPos.x, maxY);
                    System.out.println(String.format("[%s] [探索模式] 限制扫描位置到y=%d（y=47-49无目标）", 
                        getName(), maxY));
                }
            }
            
            System.out.println(String.format("[%s] [探索模式] 计算扫描目标=(%d,%d)，当前位置=(%d,%d)", 
                getName(), nextScanPos.x, nextScanPos.y, this.getX(), this.getY()));
            // 规划路径到扫描位置
            List<TWDirection> path = aStarPathfinding(new Int2D(this.getX(), this.getY()), nextScanPos);
            
            if (path != null) {
                if (path.isEmpty()) {
                    // start==goal：推进游标而不是随机走动
                    advanceScanCursor();
                    return getAnyValidDirection();
                }
                this.currentPath = new ArrayList<>(path); // 创建副本，避免直接修改
                this.currentTarget = nextScanPos;
                System.out.println(String.format("[%s] [探索模式] A*路径规划成功，路径长度=%d，下一步=%s", 
                    getName(), path.size(), path.get(0)));
                return new TWThought(TWAction.MOVE, path.get(0));
            }

            System.out.println(String.format("[%s] [探索模式] A*路径规划失败，无法到达扫描目标，使用随机方向", getName()));
        }
        
        // 如果无法规划路径，使用简单的方向移动（也要限制y坐标）
        TWThought thought = getRandomValidDirection();
        // 如果随机方向会导致y>46，检查是否有目标在y=47-49
        int nextY = this.getY();
        if (thought.getDirection() == TWDirection.S) {
            nextY = this.getY() + 1;
        }
        
        if (nextY > 46) {
            // 检查y=47-49是否有目标
            boolean hasTargetInRightArea = false;
            for (int y = 47; y <= 49; y++) {
                if (y < this.getEnvironment().getyDimension()) {
                    TWTile tile = this.memory.getNearbyTile(this.getX(), y, Double.MAX_VALUE);
                    TWHole hole = this.memory.getNearbyHole(this.getX(), y, Double.MAX_VALUE);
                    if (tile != null || hole != null) {
                        hasTargetInRightArea = true;
                        break;
                    }
                }
            }
            
            if (!hasTargetInRightArea && thought.getDirection() == TWDirection.S) {
                // 没有目标，限制移动方向（避免往南移动超过y=46）
                System.out.println(String.format("[%s] [探索模式] 限制移动方向，避免超过y=46", getName()));
                // 尝试其他方向
                TWDirection[] alternatives = {TWDirection.N, TWDirection.E, TWDirection.W};
                for (TWDirection altDir : alternatives) {
                    int testX = this.getX() + altDir.dx;
                    int testY = this.getY() + altDir.dy;
                    if (this.getEnvironment().isInBounds(testX, testY) && 
                        !this.getEnvironment().isCellBlocked(testX, testY) &&
                        !this.memory.isCellBlocked(testX, testY) &&
                        testY <= 46) {
                        thought = new TWThought(TWAction.MOVE, altDir);
                        break;
                    }
                }
            }
        }
        
        System.out.println(String.format("[%s] [探索模式] 使用随机方向移动，方向=%s", 
            getName(), thought.getDirection()));
        return thought;
    }

    /**
     * 探索阶段的简单循环检测：
     * - 两格互跳：A->B->A->B
     * - 或最近 4 次都停在同一格（原地卡住）
     *
     * 只用于“当前没有路径时”的防循环层。
     */
    private boolean detectExplorationOscillation() {
        if (positionHistory == null || positionHistory.size() < 4) {
            return false;
        }

        Int2D[] last4 = new Int2D[4];
        int size = positionHistory.size();
        int start = size - 4;
        int i = 0;
        for (Int2D p : positionHistory) {
            if (i >= start) {
                last4[i - start] = p;
            }
            i++;
        }

        if (last4[0] == null || last4[1] == null || last4[2] == null || last4[3] == null) {
            return false;
        }

        boolean isTwoCycle = last4[0].x == last4[2].x && last4[0].y == last4[2].y
                && last4[1].x == last4[3].x && last4[1].y == last4[3].y
                && !(last4[0].x == last4[1].x && last4[0].y == last4[1].y);
        if (isTwoCycle) {
            return true;
        }

        boolean allSame = last4[0].x == last4[1].x && last4[0].y == last4[1].y
                && last4[1].x == last4[2].x && last4[1].y == last4[2].y
                && last4[2].x == last4[3].x && last4[2].y == last4[3].y;
        return allSame;
    }

    /**
     * 推进犁地式扫描游标一步，避免在“当前点==目标点/不可达点”上抖动。
     */
    private void advanceScanCursor() {
        int xDim = this.getEnvironment().getxDimension();
        int yDim = this.getEnvironment().getyDimension();
        int maxY = 46;

        if (scanDirectionRight) {
            scanCol += SCAN_STEP;
            if (scanCol >= xDim) {
                scanCol = xDim - 1;
                scanRow += SCAN_STEP;
                scanDirectionRight = false;
            }
        } else {
            scanCol -= SCAN_STEP;
            if (scanCol < 0) {
                scanCol = 0;
                scanRow += SCAN_STEP;
                scanDirectionRight = true;
            }
        }

        if (scanRow > maxY) {
            scanRow = maxY;
        }

        // 若越出地图则重置扫描起点（与 calculateNextScanPosition 保持一致）
        if (scanRow >= yDim) {
            scanRow = 3;
            scanCol = 3;
            scanDirectionRight = true;
        }

        scanCol = Math.max(0, Math.min(scanCol, xDim - 1));
        scanRow = Math.max(0, Math.min(Math.min(scanRow, yDim - 1), maxY));
    }
    
    /**
     * 计算下一个犁地式扫描位置
     */
    private Int2D calculateNextScanPosition() {
        int xDim = this.getEnvironment().getxDimension();
        int yDim = this.getEnvironment().getyDimension();
        int maxY = 46; // 限制扫描最多到y=46
        
        // 如果当前位置接近目标扫描位置，更新扫描位置
        if (Math.abs(this.getX() - scanCol) <= 1 && Math.abs(this.getY() - scanRow) <= 1) {
            // 移动到下一个扫描位置
            if (scanDirectionRight) {
                scanCol += SCAN_STEP;
                if (scanCol >= xDim) {
                    // 到达右边界，换行
                    scanCol = xDim - 1;
                    scanRow += SCAN_STEP;
                    scanDirectionRight = false;
                }
            } else {
                scanCol -= SCAN_STEP;
                if (scanCol < 0) {
                    // 到达左边界，换行
                    scanCol = 0;
                    scanRow += SCAN_STEP;
                    scanDirectionRight = true;
                }
            }
            
            // 限制扫描行不超过y=46
            if (scanRow > maxY) {
                scanRow = maxY;
            }
            
            // 检查是否超出地图范围
            if (scanRow >= yDim) {
                // 重新开始扫描，从第四行第四列开始（y=3, x=3）
                scanRow = 3;
                scanCol = 3;
                scanDirectionRight = true;
            }
        }
        
        // 确保坐标在有效范围内，并限制y坐标不超过46
        scanCol = Math.max(0, Math.min(scanCol, xDim - 1));
        scanRow = Math.max(0, Math.min(scanRow, Math.min(yDim - 1, maxY)));
        
        return new Int2D(scanCol, scanRow);
    }
    
    /**
     * 刷新记忆中的障碍物（利用当前sense()信息）
     * 注意：sense()已经自动更新了记忆，这里主要是确保障碍物信息是最新的
     */
    private void refreshObstacleMemory() {
        int sensorRange = Parameters.defaultSensorRange;
        sim.field.grid.ObjectGrid2D objectGrid = this.getEnvironment().getObjectGrid();
        sim.field.grid.ObjectGrid2D memoryGrid = this.memory.getMemoryGrid();
        
        // 扫描传感器范围内的所有对象，更新障碍物记忆
        for (int dx = -sensorRange; dx <= sensorRange; dx++) {
            for (int dy = -sensorRange; dy <= sensorRange; dy++) {
                int x = this.getX() + dx;
                int y = this.getY() + dy;
                
                if (this.getEnvironment().isInBounds(x, y)) {
                    Object obj = objectGrid.get(x, y);
                    Object memObj = memoryGrid.get(x, y);
                    
                    // 如果当前位置有障碍物，更新记忆
                    if (obj instanceof TWObstacle) {
                        memoryGrid.set(x, y, obj);
                        if (memObj != obj) {
                            System.out.println(String.format("[%s] [重规划] 发现新障碍物在(%d,%d)，更新记忆", 
                                getName(), x, y));
                        }
                    } else if (memObj instanceof TWObstacle && obj == null) {
                        // 如果记忆中标记为障碍物，但实际环境中已不存在，清除记忆
                        memoryGrid.set(x, y, null);
                        System.out.println(String.format("[%s] [重规划] 障碍物在(%d,%d)已消失，清除记忆", 
                            getName(), x, y));
                    }
                }
            }
        }
    }
    
    /**
     * 评估路径成本是否可行
     * @param path 路径
     * @param targetEntity 目标实体（用于检查生命周期）
     * @return true如果路径可行，false如果成本过高
     */
    private boolean evaluatePathCost(List<TWDirection> path, TWEntity targetEntity) {
        if (path == null || path.isEmpty()) {
            return false;
        }
        
        int pathCost = path.size();
        
        // 检查1: 路径成本是否超过剩余燃料
        if (pathCost > this.getFuelLevel()) {
            System.out.println(String.format("[%s] [重规划] 路径成本(%d)超过剩余燃料(%.1f)，放弃目标", 
                getName(), pathCost, this.getFuelLevel()));
            return false;
        }
        
        // 检查2: 如果目标有生命周期，检查是否能在生命周期内到达
        if (targetEntity instanceof TWObject) {
            TWObject obj = (TWObject) targetEntity;
            double currentTime = this.getEnvironment().schedule.getTime();
            double timeLeft = obj.getTimeLeft(currentTime);
            
            if (timeLeft < pathCost) {
                System.out.println(String.format("[%s] [重规划] 路径成本(%d)超过目标剩余生命周期(%.1f)，放弃目标", 
                    getName(), pathCost, timeLeft));
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * A*路径规划算法（带重规划支持）
     */
    private List<TWDirection> aStarPathfinding(Int2D start, Int2D goal) {
        return aStarPathfinding(start, goal, null);
    }
    
    /**
     * A*路径规划算法（带重规划支持）
     * @param start 起点
     * @param goal 终点
     * @param targetEntity 目标实体（用于评估生命周期）
     * @return 路径，如果无法找到或成本过高则返回null
     */
    private List<TWDirection> aStarPathfinding(Int2D start, Int2D goal, TWEntity targetEntity) {
        // A*算法实现
        PriorityQueue<AStarNode> openSet = new PriorityQueue<>(Comparator.comparingDouble(n -> n.fCost));
        Set<Int2D> closedSet = new HashSet<>();
        Map<Int2D, AStarNode> allNodes = new HashMap<>();
        
        AStarNode startNode = new AStarNode(start, null, 0, manhattanDistance(start, goal));
        openSet.add(startNode);
        allNodes.put(start, startNode);
        
        while (!openSet.isEmpty()) {
            AStarNode current = openSet.poll();
            
            if (current.position.equals(goal)) {
                // 找到路径，重构路径
                return reconstructPath(current);
            }
            
            closedSet.add(current.position);
            
            // 检查四个方向的邻居
            TWDirection[] directions = {TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W};
            for (TWDirection dir : directions) {
                Int2D neighbor = new Int2D(current.position.x + dir.dx, current.position.y + dir.dy);
                
                // 检查边界
                if (!this.getEnvironment().isInBounds(neighbor.x, neighbor.y)) {
                    continue;
                }
                
                // 检查是否已访问
                if (closedSet.contains(neighbor)) {
                    continue;
                }
                
                // 检查是否被阻挡（根据记忆）
                if (this.memory.isCellBlocked(neighbor.x, neighbor.y)) {
                    continue;
                }
                
                // 检查实际环境是否被阻挡
                if (this.getEnvironment().isCellBlocked(neighbor.x, neighbor.y)) {
                    continue;
                }
                
                double tentativeG = current.gCost + 1;
                AStarNode neighborNode = allNodes.get(neighbor);
                
                if (neighborNode == null) {
                    neighborNode = new AStarNode(neighbor, current, tentativeG, manhattanDistance(neighbor, goal));
                    allNodes.put(neighbor, neighborNode);
                    openSet.add(neighborNode);
                } else if (tentativeG < neighborNode.gCost) {
                    neighborNode.gCost = tentativeG;
                    neighborNode.fCost = neighborNode.gCost + neighborNode.hCost;
                    neighborNode.parent = current;
                    // 重新加入优先队列
                    openSet.remove(neighborNode);
                    openSet.add(neighborNode);
                }
            }
        }
        
        // 未找到路径
        return null;
    }
    
    /**
     * 带重规划的A*路径规划
     * 如果A*失败，刷新记忆并重新规划
     */
    private List<TWDirection> aStarPathfindingWithReplan(Int2D start, Int2D goal, TWEntity targetEntity) {
        // 第一次尝试
        List<TWDirection> path = aStarPathfinding(start, goal, targetEntity);
        
        if (path != null && evaluatePathCost(path, targetEntity)) {
            replanAttempts = 0; // 重置重规划次数
            return path;
        }
        
        // A*失败或成本过高，尝试重规划
        for (int attempt = 1; attempt <= MAX_REPLAN_ATTEMPTS; attempt++) {
            System.out.println(String.format("[%s] [重规划] 第%d次重规划尝试", getName(), attempt));
            
            // 刷新记忆中的障碍物
            refreshObstacleMemory();
            
            // 重新运行A*
            path = aStarPathfinding(start, goal, targetEntity);
            
            if (path != null && evaluatePathCost(path, targetEntity)) {
                System.out.println(String.format("[%s] [重规划] 重规划成功，路径长度=%d", getName(), path.size()));
                replanAttempts = 0; // 重置重规划次数
                return path;
            }
        }
        
        System.out.println(String.format("[%s] [重规划] 重规划失败，已达到最大尝试次数(%d)，放弃目标", 
            getName(), MAX_REPLAN_ATTEMPTS));
        replanAttempts = 0; // 重置重规划次数
        return null;
    }
    
    /**
     * 重构A*路径
     */
    private List<TWDirection> reconstructPath(AStarNode node) {
        List<TWDirection> path = new ArrayList<>();
        AStarNode current = node;
        
        while (current.parent != null) {
            Int2D diff = new Int2D(
                current.position.x - current.parent.position.x,
                current.position.y - current.parent.position.y
            );
            
            // 确定方向
            TWDirection dir = null;
            if (diff.x == 1 && diff.y == 0) dir = TWDirection.E;
            else if (diff.x == -1 && diff.y == 0) dir = TWDirection.W;
            else if (diff.x == 0 && diff.y == -1) dir = TWDirection.N;
            else if (diff.x == 0 && diff.y == 1) dir = TWDirection.S;
            
            if (dir != null) {
                path.add(0, dir); // 添加到开头，因为是从目标往回追溯
            }
            
            current = current.parent;
        }
        
        return path;
    }
    
    /**
     * 曼哈顿距离
     */
    private double manhattanDistance(Int2D a, Int2D b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }
    
    /**
     * A*节点类
     */
    private static class AStarNode {
        Int2D position;
        AStarNode parent;
        double gCost; // 从起点到当前节点的实际代价
        double hCost; // 从当前节点到目标的启发式代价
        double fCost; // fCost = gCost + hCost
        
        AStarNode(Int2D position, AStarNode parent, double gCost, double hCost) {
            this.position = position;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
            this.fCost = gCost + hCost;
        }
    }
    
    /**
     * 检查记忆中是否有已知的Hole
     */
    private boolean hasKnownHole() {
        TWHole hole = this.memory.getNearbyHole(this.getX(), this.getY(), Double.MAX_VALUE);
        return hole != null;
    }
    
    /**
     * 获取随机有效方向
     */
    private TWThought getRandomValidDirection() {
        TWDirection[] directions = {TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W};
        List<TWDirection> validDirs = new ArrayList<>();
        
        for (TWDirection dir : directions) {
            int newX = this.getX() + dir.dx;
            int newY = this.getY() + dir.dy;
            
            if (this.getEnvironment().isInBounds(newX, newY) &&
                !this.getEnvironment().isCellBlocked(newX, newY) &&
                !this.memory.isCellBlocked(newX, newY)) {
                validDirs.add(dir);
            }
        }
        
        if (validDirs.isEmpty()) {
            return new TWThought(TWAction.MOVE, TWDirection.Z);
        }

        // 尽量避免“立刻走回上一格”，减少两格互跳导致的来回振荡
        if (lastMoveFrom != null && validDirs.size() > 1) {
            List<TWDirection> filtered = new ArrayList<>();
            for (TWDirection dir : validDirs) {
                int nx = this.getX() + dir.dx;
                int ny = this.getY() + dir.dy;
                if (lastMoveFrom.x == nx && lastMoveFrom.y == ny) {
                    continue;
                }
                filtered.add(dir);
            }
            if (!filtered.isEmpty()) {
                validDirs = filtered;
            }
        }

        // 再做一次短历史过滤（避免三格短环）
        if (!recentPositions.isEmpty() && validDirs.size() > 1) {
            List<TWDirection> filtered = new ArrayList<>();
            for (TWDirection dir : validDirs) {
                int nx = this.getX() + dir.dx;
                int ny = this.getY() + dir.dy;
                boolean inRecent = false;
                for (Int2D rp : recentPositions) {
                    if (rp != null && rp.x == nx && rp.y == ny) {
                        inRecent = true;
                        break;
                    }
                }
                if (!inRecent) {
                    filtered.add(dir);
                }
            }
            if (!filtered.isEmpty()) {
                validDirs = filtered;
            }
        }
        
        TWDirection chosenDir = validDirs.get(this.getEnvironment().random.nextInt(validDirs.size()));
        return new TWThought(TWAction.MOVE, chosenDir);
    }

    /**
     * 探索阶段“非等待”兜底：忽略 recentPositions/lastMoveFrom 的过滤，
     * 只要邻接格存在合法移动就走一步，避免原地卡死。
     */
    private TWThought getAnyValidDirection() {
        TWDirection[] directions = {TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W};
        List<TWDirection> validDirs = new ArrayList<>();
        for (TWDirection dir : directions) {
            int newX = this.getX() + dir.dx;
            int newY = this.getY() + dir.dy;
            if (this.getEnvironment().isInBounds(newX, newY)
                    && !this.getEnvironment().isCellBlocked(newX, newY)
                    && !this.memory.isCellBlocked(newX, newY)) {
                validDirs.add(dir);
            }
        }
        if (validDirs.isEmpty()) {
            return new TWThought(TWAction.MOVE, TWDirection.Z);
        }
        TWDirection chosenDir = validDirs.get(this.getEnvironment().random.nextInt(validDirs.size()));
        System.out.println(String.format("[%s] [探索模式] 防卡点：尝试走一步方向=%s",
                getName(), chosenDir));
        return new TWThought(TWAction.MOVE, chosenDir);
    }
    
    @Override
    protected void act(TWThought thought) {
        Int2D before = new Int2D(this.getX(), this.getY());
        super.act(thought);
        if (thought != null && thought.getAction() == TWAction.MOVE) {
            lastMoveFrom = before;
            recentPositions.addLast(new Int2D(this.getX(), this.getY()));
            while (recentPositions.size() > 3) {
                recentPositions.removeFirst();
            }
            positionHistory.addLast(new Int2D(this.getX(), this.getY()));
            while (positionHistory.size() > 8) {
                positionHistory.removeFirst();
            }

            if (thought.getDirection() == TWDirection.Z) {
                consecutiveWaitMoves++;
            } else {
                consecutiveWaitMoves = 0;
            }
        } else {
            // 其他动作也记录当前点，防止“卡住原地”的循环检测失效
            positionHistory.addLast(new Int2D(this.getX(), this.getY()));
            while (positionHistory.size() > 8) {
                positionHistory.removeFirst();
            }
            consecutiveWaitMoves = 0;
        }
    }

    @Override
    protected void appendCustomMessages(List<Message> outbox) {
        long step = getEnvironment().schedule.getSteps();
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

    // --- G7P2 接收：与演示日一致，写入工作记忆（含 SS 快照）---

    private void processIncomingCooperativeMessages() {
        for (Message m : getEnvironment().getMessages()) {
            ParsedMessage pm = parseProtocolMessage(m);
            if (pm == null || pm.from == null || pm.from.equals(getName())) {
                continue;
            }
            if (!isMessageForMe(pm)) {
                continue;
            }
            switch (pm.type) {
                case OBS_NEW_TILE:
                    rememberTeammateTile(pm.x, pm.y);
                    break;
                case OBS_NEW_HOLE:
                    rememberTeammateHole(pm.x, pm.y);
                    break;
                case OBS_OBSTACLE:
                    rememberTeammateObstacle(pm.x, pm.y);
                    break;
                case OBS_FUEL_ONCE:
                    rememberFuelStationCell(pm.x, pm.y);
                    break;
                case ACTION_PICKUP_TILE:
                    teammateTargetLeases.remove(lockCellKey(pm.x, pm.y));
                    removeTileFromMemory(pm.x, pm.y);
                    break;
                case ACTION_FILL_HOLE:
                    teammateTargetLeases.remove(lockCellKey(pm.x, pm.y));
                    removeHoleFromMemory(pm.x, pm.y);
                    break;
                case OBS_SENSOR_SNAPSHOT:
                    teammatePositionsFromSnapshot.put(pm.from, new Int2D(pm.x, pm.y));
                    applySensorSnapshot(pm.payload);
                    break;
                case TARGET_LOCK:
                    handleTeammateTargetLock(pm);
                    break;
                case TARGET_RELEASE:
                    handleTeammateTargetRelease(pm);
                    break;
                default:
                    break;
            }
        }
    }

    // --- TL / TR：与 AgentHanny 兼容的去冲突 ---

    private static String lockCellKey(int x, int y) {
        return x + "," + y;
    }

    private static final class TargetLease {
        final String owner;
        final double priority;
        final long step;
        final long expiresAtStep;

        TargetLease(String owner, double priority, long step, int ttlSteps) {
            this.owner = owner;
            this.priority = priority;
            this.step = step;
            this.expiresAtStep = step + ttlSteps;
        }
    }

    private void handleTeammateTargetLock(ParsedMessage parsed) {
        Double priority = parseTargetLockPriority(parsed.payload);
        if (priority == null) {
            return;
        }
        Integer ttl = parseTargetLockTtl(parsed.payload);
        int ttlSteps = (ttl == null) ? ownTargetLockTtlSteps : ttl.intValue();

        String key = lockCellKey(parsed.x, parsed.y);
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
        String key = lockCellKey(parsed.x, parsed.y);
        TargetLease existing = teammateTargetLeases.get(key);
        if (existing == null) {
            return;
        }
        if (existing.owner.equals(parsed.from)) {
            teammateTargetLeases.remove(key);
        }
    }

    private void pruneStaleTeammateTargetLeases(long step) {
        List<String> staleKeys = new ArrayList<String>();
        for (Map.Entry<String, TargetLease> entry : teammateTargetLeases.entrySet()) {
            if (step > entry.getValue().expiresAtStep) {
                staleKeys.add(entry.getKey());
            }
        }
        for (String key : staleKeys) {
            teammateTargetLeases.remove(key);
        }
    }

    private void ensureActiveLockStillValid(long step) {
        if (!hasActiveTargetLock()) {
            return;
        }

        if (step > lockedTargetExpiryStep) {
            releaseOwnTargetLock("local_lease_expired");
            return;
        }

        TargetLease teammate = teammateTargetLeases.get(lockCellKey(lockedTargetCell.x, lockedTargetCell.y));
        if (teammate != null) {
            if (!isPreferredLock(
                    getName(),
                    lockedTargetPriority,
                    lockedTargetStep,
                    teammate.owner,
                    teammate.priority,
                    teammate.step)) {
                releaseOwnTargetLock("lost_priority");
                return;
            }
        }

        if (!isInSensorRange(lockedTargetCell.x, lockedTargetCell.y)) {
            return;
        }

        TWEntity obj = (TWEntity) getEnvironment().getObjectGrid().get(lockedTargetCell.x, lockedTargetCell.y);
        if (lockedTargetKind == LockKind.HOLE) {
            if (!(obj instanceof TWHole)) {
                releaseOwnTargetLock("target_changed_or_missing");
            }
        } else if (lockedTargetKind == LockKind.TILE) {
            if (!(obj instanceof TWTile)) {
                releaseOwnTargetLock("target_changed_or_missing");
            }
        }
    }

    private boolean isInSensorRange(int x, int y) {
        int r = Parameters.defaultSensorRange;
        return Math.abs(x - getX()) <= r && Math.abs(y - getY()) <= r;
    }

    private boolean tryAcquireOrKeepLock(LockKind kind, int x, int y, long step) {
        if (isOwnLockAt(kind, x, y)) {
            return true;
        }

        double priority = computeLockPriority(kind, x, y, step);
        if (isBlockedByPreferredTeammateLock(x, y, priority, step)) {
            return false;
        }

        releaseOwnTargetLock("switch_target");
        lockedTargetCell = new Int2D(x, y);
        lockedTargetKind = kind;
        lockedTargetPriority = priority;
        lockedTargetStep = step;
        lockedTargetExpiryStep = step + ownTargetLockTtlSteps;
        publishProtocolMessage(
                CommType.TARGET_LOCK,
                x,
                y,
                encodeTargetLockPayload(priority, ownTargetLockTtlSteps));
        return true;
    }

    private boolean isBlockedByPreferredTeammateLock(int x, int y, double myPriority, long myStep) {
        TargetLease teammate = teammateTargetLeases.get(lockCellKey(x, y));
        if (teammate == null) {
            return false;
        }

        return !isPreferredLock(
                getName(),
                myPriority,
                myStep,
                teammate.owner,
                teammate.priority,
                teammate.step);
    }

    private boolean isLeasePreferred(TargetLease incoming, TargetLease existing) {
        return isPreferredLock(
                incoming.owner,
                incoming.priority,
                incoming.step,
                existing.owner,
                existing.priority,
                existing.step);
    }

    private boolean isPreferredLock(
            String ownerA,
            double priorityA,
            long stepA,
            String ownerB,
            double priorityB,
            long stepB) {

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

    private double computeLockPriority(LockKind kind, int x, int y, long step) {
        long seed = 1469598103934665603L;
        seed ^= (long) getName().hashCode();
        seed *= 1099511628211L;
        seed ^= (long) kind.ordinal();
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

    private static int computeDefaultTargetLockTtl(TWEnvironment env) {
        int span = Math.max(1, env.getxDimension() + env.getyDimension());
        int ttl = span / 6;
        if (ttl < LOCK_TTL_MIN_STEPS) {
            return LOCK_TTL_MIN_STEPS;
        }
        if (ttl > LOCK_TTL_MAX_STEPS) {
            return LOCK_TTL_MAX_STEPS;
        }
        return ttl;
    }

    private boolean hasActiveTargetLock() {
        return lockedTargetCell != null && lockedTargetKind != null;
    }

    private boolean shouldRenewTargetLock(long step) {
        if (!hasActiveTargetLock()) {
            return false;
        }
        if (lockedTargetExpiryStep < 0) {
            return true;
        }
        return step >= (lockedTargetExpiryStep - LOCK_RENEW_BEFORE_STEPS);
    }

    private boolean isOwnLockAt(LockKind kind, int x, int y) {
        return hasActiveTargetLock()
                && lockedTargetKind == kind
                && lockedTargetCell.x == x
                && lockedTargetCell.y == y;
    }

    private void releaseOwnTargetLock(String reason) {
        if (!hasActiveTargetLock()) {
            return;
        }

        publishProtocolMessage(
                CommType.TARGET_RELEASE,
                lockedTargetCell.x,
                lockedTargetCell.y,
                (reason == null) ? "" : reason);

        lockedTargetCell = null;
        lockedTargetKind = null;
        lockedTargetPriority = 0.0;
        lockedTargetStep = -1L;
        lockedTargetExpiryStep = -1L;
    }

    private List<TWHole> listHolesInMemorySortedByDistance() {
        ObjectGrid2D mg = memory.getMemoryGrid();
        int xd = getEnvironment().getxDimension();
        int yd = getEnvironment().getyDimension();
        List<TWHole> out = new ArrayList<TWHole>();
        for (int x = 0; x < xd; x++) {
            for (int y = 0; y < yd; y++) {
                Object o = mg.get(x, y);
                if (o instanceof TWHole) {
                    out.add((TWHole) o);
                }
            }
        }
        final int ax = getX();
        final int ay = getY();
        Collections.sort(out, new Comparator<TWHole>() {
            @Override
            public int compare(TWHole a, TWHole b) {
                int da = Math.abs(a.getX() - ax) + Math.abs(a.getY() - ay);
                int db = Math.abs(b.getX() - ax) + Math.abs(b.getY() - ay);
                return Integer.compare(da, db);
            }
        });
        return out;
    }

    private TWHole selectHoleForFill(long step) {
        for (TWHole h : listHolesInMemorySortedByDistance()) {
            Object g = getEnvironment().getObjectGrid().get(h.getX(), h.getY());
            if (!(g instanceof TWHole)) {
                continue;
            }
            if (tryAcquireOrKeepLock(LockKind.HOLE, h.getX(), h.getY(), step)) {
                return h;
            }
        }
        return null;
    }

    private void rememberFuelStationCell(int x, int y) {
        ObjectGrid2D grid = getEnvironment().getObjectGrid();
        Object obj = grid.get(x, y);
        if (obj instanceof TWFuelStation) {
            memory.getMemoryGrid().set(x, y, (TWFuelStation) obj);
        }
    }

    private void rememberTeammateTile(int x, int y) {
        Object envObj = getEnvironment().getObjectGrid().get(x, y);
        if (envObj instanceof TWTile) {
            memory.getMemoryGrid().set(x, y, (TWTile) envObj);
            return;
        }
        double t = getEnvironment().schedule.getTime();
        memory.getMemoryGrid().set(x, y, new TWTile(x, y, getEnvironment(), t, t + Parameters.lifeTime));
    }

    private void rememberTeammateHole(int x, int y) {
        Object envObj = getEnvironment().getObjectGrid().get(x, y);
        if (envObj instanceof TWHole) {
            memory.getMemoryGrid().set(x, y, (TWHole) envObj);
            return;
        }
        double t = getEnvironment().schedule.getTime();
        memory.getMemoryGrid().set(x, y, new TWHole(x, y, getEnvironment(), t, t + Parameters.lifeTime));
    }

    private void rememberTeammateObstacle(int x, int y) {
        Object envObj = getEnvironment().getObjectGrid().get(x, y);
        if (envObj instanceof TWObstacle) {
            memory.getMemoryGrid().set(x, y, (TWObstacle) envObj);
            return;
        }
        double t = getEnvironment().schedule.getTime();
        memory.getMemoryGrid().set(x, y, new TWObstacle(x, y, getEnvironment(), t, t + Parameters.lifeTime));
    }

    private void removeTileFromMemory(int x, int y) {
        Object mem = memory.getMemoryGrid().get(x, y);
        if (mem instanceof TWTile) {
            memory.removeObject((TWTile) mem);
        } else {
            memory.getMemoryGrid().set(x, y, null);
        }
    }

    private void removeHoleFromMemory(int x, int y) {
        Object mem = memory.getMemoryGrid().get(x, y);
        if (mem instanceof TWHole) {
            memory.removeObject((TWHole) mem);
        } else {
            memory.getMemoryGrid().set(x, y, null);
        }
    }

    private void applySensorSnapshot(String payload) {
        for (SnapshotItem item : SensorSnapshotCodec.decode(payload)) {
            if ("E".equals(item.code)) {
                Object mem = memory.getMemoryGrid().get(item.x, item.y);
                if (!(mem instanceof TWFuelStation)) {
                    memory.getMemoryGrid().set(item.x, item.y, null);
                }
                continue;
            }
            if ("T".equals(item.code)) {
                rememberTeammateTile(item.x, item.y);
            } else if ("H".equals(item.code)) {
                rememberTeammateHole(item.x, item.y);
            } else if ("O".equals(item.code)) {
                rememberTeammateObstacle(item.x, item.y);
            }
        }
    }

    /**
     * 打破六克隆完全同相位：每人犁地点起算位置略不同，减少起步就同轨。
     */
    private void applyExploreScanPhaseOffsetFromName(String agentName, TWEnvironment env) {
        int xd = env.getxDimension();
        int yd = env.getyDimension();
        int h = Math.abs(agentName.hashCode());
        int colSpan = Math.max(2, Math.min(SCAN_STEP * 2, Math.max(2, xd / 2)));
        int rowSpan = Math.max(2, Math.min(SCAN_STEP * 2, Math.max(2, Math.min(yd / 2, 24))));
        scanCol = 3 + (h % colSpan);
        scanRow = 3 + ((h / 29) % rowSpan);
        scanCol = Math.max(0, Math.min(xd - 1, scanCol));
        scanRow = Math.max(0, Math.min(Math.min(yd - 1, 46), scanRow));
    }

    /** 主阶段探索：优先把扫描锚点拉回到开局 Manager 分配的矩形内，减轻全图往同一条犁沟挤。 */
    private Int2D clipPointToAssignedZone(Int2D p) {
        ZoneAssignment z = getZoneAssignment();
        if (z == null || p == null) {
            return p;
        }
        int x = Math.max(z.x1, Math.min(z.x2, p.x));
        int y = Math.max(z.y1, Math.min(z.y2, p.y));
        return new Int2D(x, y);
    }

    /**
     * 软排斥：若扫描目标与某队友过近（曼哈顿小于约两格传感器直径），沿远离方向微量平移目标。
     * 数据来自 SS 里的发送者坐标；不协议、不 TL，仅探索用。
     */
    private Int2D nudgeExploreAwayFromTeammates(Int2D p) {
        if (p == null || teammatePositionsFromSnapshot.isEmpty()) {
            return p;
        }
        int r = Parameters.defaultSensorRange;
        int minManhattan = 2 * r + 1;
        int xDim = getEnvironment().getxDimension();
        int yDim = getEnvironment().getyDimension();
        int maxY = 46;
        Int2D cur = new Int2D(p.x, p.y);

        for (int attempt = 0; attempt < 4; attempt++) {
            int pushX = 0;
            int pushY = 0;
            for (Map.Entry<String, Int2D> e : teammatePositionsFromSnapshot.entrySet()) {
                Int2D m = e.getValue();
                if (m == null) {
                    continue;
                }
                int dist = Math.abs(cur.x - m.x) + Math.abs(cur.y - m.y);
                if (dist < minManhattan) {
                    int sx = Integer.signum(cur.x - m.x);
                    int sy = Integer.signum(cur.y - m.y);
                    if (sx == 0 && sy == 0) {
                        sx = 1;
                    }
                    pushX += sx;
                    pushY += sy;
                }
            }
            if (pushX == 0 && pushY == 0) {
                break;
            }
            pushX = Integer.signum(pushX) * Math.min(2, Math.abs(pushX));
            pushY = Integer.signum(pushY) * Math.min(2, Math.abs(pushY));
            int nx = cur.x + pushX;
            int ny = cur.y + pushY;
            nx = Math.max(0, Math.min(xDim - 1, nx));
            ny = Math.max(0, Math.min(Math.min(yDim - 1, maxY), ny));
            cur = new Int2D(nx, ny);
        }
        return cur;
    }

    // --- 分区.bootstrap：进区前先去区域中心，进区后按传感器步长之字形扫（与 AgentHanny 对齐）---

    /**
     * Bootstrap 与主策略一致：当前格可捡 tile / 可填洞则优先处理（不阻断进区大方向）。
     */
    private TWThought tryOpportunisticTileOrHoleOnCell() {
        TWEntity here = (TWEntity) getEnvironment().getObjectGrid().get(getX(), getY());
        if (here instanceof TWHole && hasTile()) {
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (here instanceof TWTile && carriedTiles.size() < 3) {
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }
        return null;
    }

    private TWThought bootstrapZoneSearch(long step) {
        if (step == 0L) {
            return waitThought();
        }
        TWThought op = tryOpportunisticTileOrHoleOnCell();
        if (op != null) {
            return op;
        }
        ZoneAssignment zone = getZoneAssignment();
        if (zone == null) {
            return waitThought();
        }
        if (!zone.contains(this.getX(), this.getY())) {
            resetZoneSweepIfNeeded(zone);
            clearPlan();
            return stepToward(zone.center());
        }
        return exploreWithinZone(zone);
    }

    private TWThought exploreWithinZone(ZoneAssignment zone) {
        TWThought op = tryOpportunisticTileOrHoleOnCell();
        if (op != null) {
            return op;
        }
        resetZoneSweepIfNeeded(zone);
        for (int attempts = 0; attempts < 6; attempts++) {
            if (zoneSweepTarget == null || atPosition(zoneSweepTarget)) {
                zoneSweepTarget = nextZoneSweepTarget(zone);
            }
            TWThought thought = stepToward(zoneSweepTarget);
            if (!isWaitThought(thought)) {
                return thought;
            }
            zoneSweepTarget = nextZoneSweepTarget(zone);
            clearPlan();
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
    }

    private Int2D nextZoneSweepTarget(ZoneAssignment zone) {
        int stride = Math.max(1, (Parameters.defaultSensorRange * 2) + 1);
        if (zoneSweepRight) {
            int nextX = zoneSweepX + stride;
            if (nextX <= zone.x2) {
                zoneSweepX = nextX;
            } else {
                zoneSweepX = zone.x2;
                zoneSweepY += stride;
                zoneSweepRight = false;
            }
        } else {
            int nextX = zoneSweepX - stride;
            if (nextX >= zone.x1) {
                zoneSweepX = nextX;
            } else {
                zoneSweepX = zone.x1;
                zoneSweepY += stride;
                zoneSweepRight = true;
            }
        }
        if (zoneSweepY > zone.y2) {
            zoneSweepY = zone.y1;
        }
        return new Int2D(zoneSweepX, zoneSweepY);
    }

    private boolean isWaitThought(TWThought thought) {
        return thought != null
                && thought.getAction() == TWAction.MOVE
                && thought.getDirection() == TWDirection.Z;
    }

    private boolean atPosition(Int2D target) {
        return target != null && this.getX() == target.x && this.getY() == target.y;
    }
}
