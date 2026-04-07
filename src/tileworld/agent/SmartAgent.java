package tileworld.agent;

import java.util.*;
import sim.field.grid.ObjectGrid2D;
import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.agent.classdefines.MemoryObjectType;
import tileworld.agent.classdefines.MemorySideCard;
import tileworld.agent.classdefines.MemorySideCardEntry;
import tileworld.environment.*;
import tileworld.exceptions.CellBlockedException;
import sim.util.Int2D;
import tileworld.agent.classdefines.MemorySideCardEntry;


public class SmartAgent extends Group7AgentBase {

    // 配置
    private static final double FUEL_FORCE_REFuel = 150;   // 强制加油
    private static final double FUEL_WARNING = 200;       // 预警层
    private static final int RECENT_MAX = 10;
    private static final double RANDOM_STEP = 0.05;
    private static final int LOCAL_RANGE = 12;
    private static final int ASTAR_MAX_NODES = 150;
    private static final int SAFETY_FUEL_BUFFER = 20;     // 往返安全余量

    // 状态 
    private final Deque<int[]> recentPositions = new ArrayDeque<>();
    private final Set<int[]> knownFuelStations = new HashSet<>(); // 记忆加油站
    private final Random rand = new Random();
    private boolean shouldRefuel = false;

    // 网格搜索状态
    private boolean gridSearchInitialized = false;
    private List<Integer> gridCenterXs = new ArrayList<>();
    private List<Integer> gridCenterYs = new ArrayList<>();
    private int currentGridIndex = 0;

    private TWObject currentLockedTarget = null;
    private long lockExpiryTime = 0;
    private final long LOCK_DURATION = 20;

    // lock
    private final int ownTargetLockTtlSteps;
    private final Map<String, TargetLease> teammateTargetLeases = new HashMap<String, TargetLease>();
    private static final int LOCK_TTL_MIN_STEPS = 8;
    private static final int LOCK_TTL_MAX_STEPS = 20;
    private static final double LOCK_PRIORITY_EPSILON = 1e-9;
    private static final int LOCK_RENEW_BEFORE_STEPS = 2;

    private Int2D lockedTargetCell = null;
    private double lockedTargetPriority = 0.0;
    private MemoryObjectType lockedTargetType = null;
    private long lockedTargetStep = -1L;
    private long lockedTargetExpiryStep = -1L;

    // 构造
    public SmartAgent(int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super("SmartAgent",xpos, ypos, env, fuelLevel);
        this.ownTargetLockTtlSteps = computeDefaultTargetLockTtl(env);
        enableZoneCoordination();
    }

    // 必须覆写的方法
    @Override
    public String getName() {
        return super.getName();
    }

    //@Override
    //public void communicate() { }

    @Override
    protected void appendCustomMessages(List<Message> outbox) {
        // Renew only when lease is close to expiry to reduce traffic.
        long step = this.getEnvironment().schedule.getSteps();
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
    
    @Override
    protected TWThought think() {
        sense();
        long step = this.getEnvironment().schedule.getSteps();
        processIncomingMessages(step);
        processZoneCoordinationInThink();
        int x = getX();
        int y = getY();
        double fuel = getFuelLevel();

        // 记录最近走过的位置
        recentPositions.offer(new int[]{x, y});
        if (recentPositions.size() > RECENT_MAX) recentPositions.poll();

        // 记忆当前视野内的加油站（新增）
        memorizeVisibleFuelStations();

        //锁定
        boolean fullOfTiles = carriedTiles.size() >= 3;
        if (!fullOfTiles) {
            TWTile targetTile = (TWTile) memory.getClosestObjectInSensorRange(TWTile.class);
            if (targetTile != null) {
                MemorySideCardEntry tileEntry = new MemorySideCardEntry(MemoryObjectType.TILE, targetTile.getX(), targetTile.getY(), step, step, false);
                tryAcquireOrKeepTargetLock(tileEntry, step); // 抢占/保持锁
            }
        }
        // 检测视野内的Hole并尝试锁定（仅当携带瓷砖时）
        if (!carriedTiles.isEmpty()) {
            TWHole targetHole = (TWHole) memory.getClosestObjectInSensorRange(TWHole.class);
            if (targetHole != null) {
                MemorySideCardEntry holeEntry = new MemorySideCardEntry(MemoryObjectType.HOLE, targetHole.getX(), targetHole.getY(), step, step, false);
                tryAcquireOrKeepTargetLock(holeEntry, step); // 抢占/保持锁
            }
        }

        // 已知加油站
        if (!knownFuelStations.isEmpty()) {

            // 强制加油（<150）
            if (fuel < FUEL_FORCE_REFuel) {
                if (this.getEnvironment().inFuelStation(this) && shouldRefuel) {
                    System.out.println(String.format("%s 已在燃料站位置，执行加油", getName()));
                    return new TWThought(TWAction.REFUEL, TWDirection.Z);
                }
                return thinkRefuel();
            }

            // 预警模式（<200）：只做近距离任务
            // 正常模式（>=200）
            boolean warningMode = fuel < FUEL_WARNING;

            // 有瓷砖/洞 → 拾取/填补
            boolean hasTile = memory.getClosestObjectInSensorRange(TWTile.class) != null;
            if (fullOfTiles) {
                hasTile = false;
            }
            boolean hasHole = memory.getClosestObjectInSensorRange(TWHole.class) != null;
            if (hasTile || hasHole) {
                return thinkPickFill(warningMode);
            }

            // 视野有加油站 → 仅预警模式加油
            if (warningMode) {
                boolean hasFuel = memory.getClosestObjectInSensorRange(TWFuelStation.class) != null;
                if (hasFuel) {
                    return thinkRefuel();
                }
            }

            // 探索（预警模式下缩小范围）
            releaseOwnTargetLock("explore_no_candidate");
            return thinkExplore(warningMode);
        } else {
            
            return thinkGridSearch();
        }

        
    }

    @Override
    protected void act(TWThought thought) {
        System.out.println("Score: " + this.score);
        if (thought == null) thought = safeMove();

        try {
            switch (thought.getAction()) {
                case MOVE:
                    move(thought.getDirection());
                    break;
                case PICKUP:
                    TWTile t = (TWTile) getEnvironment().getObjectGrid().get(getX(), getY());
                    if (t != null && carriedTiles.size() < 3) pickUpTile(t);
                    releaseOwnTargetLock("pick");
                    break;
                case PUTDOWN:
                    TWHole h = (TWHole) getEnvironment().getObjectGrid().get(getX(), getY());
                    if (h != null && !carriedTiles.isEmpty()) putTileInHole(h);
                    releaseOwnTargetLock("put");
                    break;
                case REFUEL:
                    if (getEnvironment().inFuelStation(this)) refuel();
                    break;
            }
        } catch (Exception e) {
            safeMove();
        }
    }

    private TWThought thinkGridSearch() {
        final int BLOCK_SIZE = 7;
        
        ZoneAssignment zone = getZoneAssignment();
        if (zone == null) {
            return waitThought();
        }

        // 初始化：生成蛇形往返的所有网格中心点（正行→反行→正行）
        if (!gridSearchInitialized) {
            gridCenterXs.clear();
            gridCenterYs.clear();
            currentGridIndex = 0;

            int mapW = zone.x2;
            int mapH = zone.y2;
            boolean reverse = false;

            // 逐行生成，蛇形往返
            for (int blockY = zone.y1; blockY < mapH; blockY += BLOCK_SIZE) {
                List<Integer> rowCentersX = new ArrayList<>();
                List<Integer> rowCentersY = new ArrayList<>(); // 配合可能的 Y 轴偏移

                int currentBlockHeight = Math.min(BLOCK_SIZE, mapH - blockY);
                int centerY = blockY + (currentBlockHeight / 2);

                for (int blockX = zone.x1; blockX < mapW; blockX += BLOCK_SIZE) {
                    int currentBlockWidth = Math.min(BLOCK_SIZE, mapW - blockX);
                    int centerX = blockX + (currentBlockWidth / 2);

                    // 防止中心点恰好是障碍物导致丢弃整个 7x7 视野区
                    // 由内向外螺旋寻找最近的非障碍物有效点（搜索半径最大为 3）
                    int validX = -1, validY = -1;
                    outerLoop:
                    for (int r = 0; r <= 3; r++) { 
                        for (int dx = -r; dx <= r; dx++) {
                            for (int dy = -r; dy <= r; dy++) {
                                // 仅检查当前圈层，避免重复检查
                                if (Math.max(Math.abs(dx), Math.abs(dy)) != r) continue;
                                
                                int nx = centerX + dx;
                                int ny = centerY + dy;
                                // 过滤：目标点不能是障碍物/边界/洞
                                if (!isOutOfBounds(nx, ny) && !isObstacle(nx, ny) && !isHole(nx, ny)) {
                                    validX = nx;
                                    validY = ny;
                                    break outerLoop;
                                }
                            }
                        }
                    }

                    // 只要找到了有效点，就加入路径点列表
                    if (validX != -1) {
                        rowCentersX.add(validX);
                        rowCentersY.add(validY);
                    }
                }

                // 蛇形搜索：偶数行正序，奇数行反序
                if (reverse) {
                    Collections.reverse(rowCentersX);
                    Collections.reverse(rowCentersY);
                }
                reverse = !reverse;

                // 加入总列表
                for (int i = 0; i < rowCentersX.size(); i++) {
                    gridCenterXs.add(rowCentersX.get(i));
                    gridCenterYs.add(rowCentersY.get(i));
                }
            }

            gridSearchInitialized = true;
        }

        // 找到加油站 → 停止搜索
        if (!knownFuelStations.isEmpty()) {
            gridSearchInitialized = false;
            currentGridIndex = 0;
            return think();
        }

        // 全部扫描完毕 → 重置
        if (currentGridIndex >= gridCenterXs.size()) {
            gridSearchInitialized = false;
            currentGridIndex = 0;
            return safeMove();
        }

        while (currentGridIndex < gridCenterXs.size()) {
            int targetX = gridCenterXs.get(currentGridIndex);
            int targetY = gridCenterYs.get(currentGridIndex);

            // 动态判断允许的绕路误差
            // 判定该目标点是否是负责扫描地图边缘的“关键点”（距离边界视野不足或刚等于视野）
            boolean nearEdge = (targetX - zone.x1 <= 3) || (zone.x2 - targetX <= 3) ||
                               (targetY - zone.y1 <= 3) || (zone.y2 - targetY <= 3);

            // 如果是边缘点，误差必须严格为 0，否则“提前掉头”会导致 agent 看不到最外层的墙根(x=0 等)；
            // 如果是中间点，继续允许 2 的绕路误差。
            double allowedDist = nearEdge ? 0 : 2;

            double distToTarget = manhattan(getX(), getY(), targetX, targetY);
            
            // 已到达目标点要求距离
            if (distToTarget <= allowedDist) {
                memorizeVisibleFuelStations(); // 扫描视野
                currentGridIndex++; // 切换下一个目标点
                continue;
            }

            // 当前目标点如果在区域内不可达，尝试使用左/上/右三个相邻点，
            // 这适用于障碍物仅占一格的情况，避免直接跳过整行。
            if (!isReachableInZone(targetX, targetY, zone)) {
                int[] alt = resolveBlockedCenterTarget(targetX, targetY, zone);
                if (alt != null) {
                    return hierarchicalPlan(alt[0], alt[1]);
                }
                currentGridIndex++;
                continue;
            }

            // 使用 hierarchicalPlan 规划路径
            return hierarchicalPlan(targetX, targetY);
        }

        return safeMove();
    }

    // 记忆加油站
    private void memorizeVisibleFuelStations() {
        int x = getX();
        int y = getY();
        int sensorRange = 3; // 7x7视野：中心±3

        // 遍历7x7视野内所有格子
        for (int dx = -sensorRange; dx <= sensorRange; dx++) {
            for (int dy = -sensorRange; dy <= sensorRange; dy++) {
                int nx = x + dx;
                int ny = y + dy;
                if (isOutOfBounds(nx, ny)) continue;

                Object obj = getEnvironment().getObjectGrid().get(nx, ny);
                if (obj instanceof TWFuelStation) {
                    int fx = nx;
                    int fy = ny;
                    // 检查是否已记忆，避免重复添加
                    boolean exists = false;
                    for (int[] p : knownFuelStations) {
                        if (p[0] == fx && p[1] == fy) {
                            exists = true;
                            break;
                        }
                    }
                    if (!exists) {
                        knownFuelStations.add(new int[]{fx, fy});
                        //outbox.add(encodeProtocolMessage(CommType.OBS_FUEL_ONCE, fx, fy, ""));
                    }
                }
            }
        }
    }

    // 获取最近的记忆加油站
    private int[] getClosestKnownFuelStation() {
        int x = getX(), y = getY();
        int[] best = null;
        double minDist = Double.MAX_VALUE;

        for (int[] p : knownFuelStations) {
            double dist = Math.abs(p[0] - x) + Math.abs(p[1] - y);
            if (dist < minDist) {
                minDist = dist;
                best = p;
            }
        }
        return best;
    }

    // 往返油量检查
    private boolean hasEnoughFuelForRoundTrip(int tx, int ty) {
        int[] fuelStation = getClosestKnownFuelStation();
        if (fuelStation == null) return true; // 无加油站 → 允许

        int x = getX(), y = getY();
        double toTarget = Math.abs(x - tx) + Math.abs(y - ty);
        double targetToFuel = Math.abs(tx - fuelStation[0]) + Math.abs(ty - fuelStation[1]);
        double total = toTarget + targetToFuel + SAFETY_FUEL_BUFFER;

        return getFuelLevel() >= total;
    }

    // 加油模式（使用记忆）
    private TWThought thinkRefuel() {
        shouldRefuel = true;
        // 优先视野内
        TWFuelStation s = (TWFuelStation) memory.getClosestObjectInSensorRange(TWFuelStation.class);
        if (s != null) {
            return hierarchicalPlan(s.getX(), s.getY());
        }

        // 视野无 → 使用记忆
        int[] closest = getClosestKnownFuelStation();
        if (closest != null) {
            return hierarchicalPlan(closest[0], closest[1]);
        }

        // 完全不知道 → 随机安全
        return safeMove();
    }

    // 捡瓷砖/补洞（预警+油量检查）
    private TWThought thinkPickFill(boolean warningMode) {
        int x = getX(), y = getY();

        //检查锁
        if (hasActiveTargetLock()) {
            int tx = lockedTargetCell.x;
            int ty = lockedTargetCell.y;
            // 验证锁定目标是否仍有效
            if (lockedTargetType == MemoryObjectType.TILE && isTileExist(tx, ty)) {
                if (warningMode && (Math.abs(tx - x) + Math.abs(ty - y) > 15)) {
                    return thinkRefuel();
                }
                if (hasEnoughFuelForRoundTrip(tx, ty)) {
                    return hierarchicalPlan(tx, ty);
                } else {
                    return thinkRefuel();
                }
            } else if (lockedTargetType == MemoryObjectType.HOLE && isHoleExist(tx, ty) && !carriedTiles.isEmpty()) {
                if (warningMode && (Math.abs(tx - x) + Math.abs(ty - y) > 15)) {
                    return thinkRefuel();
                }
                if (hasEnoughFuelForRoundTrip(tx, ty)) {
                    return hierarchicalPlan(tx, ty);
                } else {
                    return thinkRefuel();
                }
            } else {
                // 锁定目标失效，释放锁
                releaseOwnTargetLock("target_invalid");
            }
        }

        // 有瓷砖 → 拾取
        if (carriedTiles.size() < 3) {
            TWTile targetTile = (TWTile) memory.getClosestObjectInSensorRange(TWTile.class);
            if (targetTile != null) {
                int tx = targetTile.getX(), ty = targetTile.getY();

                // 被队友锁定 → 直接去探索
                String targetCell = MemorySideCard.cellKey(tx, ty);
                if (teammateTargetLeases.containsKey(targetCell) && !teammateTargetLeases.get(targetCell).owner.equals(this.getName())) {
                    return thinkExplore(warningMode);
                }

                // 实时校验：Tile是否还存在（未被其他Agent捡走）
                if (!isTileExist(tx, ty)) {
                    memory.removeObject(targetTile);
                    return thinkExplore(warningMode);
                }
                // 预警模式+油量校验
                if (warningMode && (Math.abs(tx - x) + Math.abs(ty - y) > 15)) {
                    return thinkRefuel();
                }
                if (hasEnoughFuelForRoundTrip(tx, ty)) {
                    return hierarchicalPlan(tx, ty);
                } else {
                    return thinkRefuel();
                }
            }
        }

        // 有洞 → 填补
        TWHole targetHole = (TWHole) memory.getClosestObjectInSensorRange(TWHole.class);
        if (targetHole != null && !carriedTiles.isEmpty()) {
            int hx = targetHole.getX(), hy = targetHole.getY();

            // 被队友锁定 → 直接去探索
            String targetCell = MemorySideCard.cellKey(hx, hy);
            if (teammateTargetLeases.containsKey(targetCell) && !teammateTargetLeases.get(targetCell).owner.equals(this.getName())) {
                return thinkExplore(warningMode);
            }

            // 实时校验：Hole是否还存在（未被其他Agent填补）
            if (!isHoleExist(hx, hy)) {
                memory.removeObject(targetHole);
                return thinkExplore(warningMode);
            }
            // 预警模式+油量校验
            if (warningMode && (Math.abs(hx - x) + Math.abs(hy - y) > 15)) {
                return thinkRefuel();
            }
            if (hasEnoughFuelForRoundTrip(hx, hy)) {
                return hierarchicalPlan(hx, hy);
            } else {
                return thinkRefuel();
            }
        }

        return safeMove();
    }
    
    private TWThought thinkExplore(boolean warningMode) {
        int x = getX(), y = getY();

        List<TWDirection> valid = new ArrayList<>();
        List<TWDirection> wallDirs = new ArrayList<>();

        for (TWDirection d : TWDirection.values()) {
            int nx = x + d.dx;
            int ny = y + d.dy;

            // 越界 / 障碍物 → 跳过
            if (isOutOfBounds(nx, ny) || isObstacle(nx, ny)) {
                continue;
            }

            // 最近走过 → 跳过（防止回头卡）
            if (isRecentlyVisited(nx, ny)) {
                continue;
            }

            // 预警模式：不往远离加油站的方向走
            if (warningMode) {
                int[] fs = getClosestKnownFuelStation();
                if (fs != null) {
                    int dx1 = x - fs[0];
                    int dy1 = y - fs[1];
                    int distNow = (dx1 < 0 ? -dx1 : dx1) + (dy1 < 0 ? -dy1 : dy1);

                    int dx2 = nx - fs[0];
                    int dy2 = ny - fs[1];
                    int distNext = (dx2 < 0 ? -dx2 : dx2) + (dy2 < 0 ? -dy2 : dy2);

                    if (distNext > distNow) {
                        continue;
                    }
                }
            }

            valid.add(d);

            // 贴墙/贴洞 → 优先
            if (isAdjacentToWallOrHole(nx, ny)) {
                wallDirs.add(d);
            }
        }

        // 优先贴墙走
        if (!wallDirs.isEmpty()) {
            return new TWThought(TWAction.MOVE, wallDirs.get(0));
        }

        // 其次走没走过的
        if (!valid.isEmpty()) {
            return new TWThought(TWAction.MOVE, valid.get(0));
        }

        // 兜底
        return safeMove();
    }

    // 路径规划 
    private TWThought hierarchicalPlan(int tx, int ty) {
        String targetCell = MemorySideCard.cellKey(tx, ty);
        if (teammateTargetLeases.containsKey(targetCell) && !teammateTargetLeases.get(targetCell).owner.equals(this.getName())) {
            return thinkExplore(true);
        }

        int x = getX(), y = getY();
        if (x == tx && y == ty) {
            // 目标是洞且携带瓷砖 → 放置
            if (isHoleExist(x, y) && !carriedTiles.isEmpty()) {
                releaseOwnTargetLock("put");
                return new TWThought(TWAction.PUTDOWN, null);
            }
            // 目标是瓷砖且未满载 → 拾取
            if (isTileExist(x, y) && carriedTiles.size() < 3) {
                releaseOwnTargetLock("pick");
                return new TWThought(TWAction.PICKUP, null);
            }
            // 目标无效（如瓷砖已被捡走）→ 释放锁+探索
            releaseOwnTargetLock("target_invalid");
            return thinkExplore(false);
        }

        // 原有寻路逻辑
        TWDirection d = limitedAStar(tx, ty);
        if (d != null) return new TWThought(TWAction.MOVE, d);
        d = greedyDir(tx, ty);
        if (d != null) return new TWThought(TWAction.MOVE, d);
        d = bfsStep(tx, ty, 2);
        if (d != null) return new TWThought(TWAction.MOVE, d);
        return safeMove();
    }

    // A* / 贪心 / BFS
    private TWDirection limitedAStar(int tx, int ty) {
        int x0 = getX(), y0 = getY();
        if (x0 == tx && y0 == ty) return null;
        Set<Long> closed = new HashSet<>();
        PriorityQueue<Node> open = new PriorityQueue<>(Comparator.comparingDouble(n -> n.f));
        open.add(new Node(x0, y0, null, 0, manhattan(x0, y0, tx, ty)));
        int count = 0;

        while (!open.isEmpty() && count < ASTAR_MAX_NODES) {
            count++;
            Node cur = open.poll();
            if (cur.x == tx && cur.y == ty) return reconstructFirstDir(cur);
            closed.add(key(cur.x, cur.y));

            for (TWDirection d : TWDirection.values()) {
                int nx = cur.x + d.dx;
                int ny = cur.y + d.dy;
                if (isOutOfBounds(nx, ny) || isObstacle(nx, ny)) continue;
                if (closed.contains(key(nx, ny))) continue;
                if (Math.abs(nx - x0) > LOCAL_RANGE || Math.abs(ny - y0) > LOCAL_RANGE) continue;
                open.add(new Node(nx, ny, cur, cur.g + 1, manhattan(nx, ny, tx, ty)));
            }
        }
        return null;
    }

    private TWDirection greedyDir(int tx, int ty) {
        int x0 = getX(), y0 = getY();
        if (x0 == tx && y0 == ty) return null;
        TWDirection best = null;
        double minDist = 9999;
        for (TWDirection d : TWDirection.values()) {
            int nx = getX() + d.dx;
            int ny = getY() + d.dy;
            if (isOutOfBounds(nx, ny) || isObstacle(nx, ny)) continue;
            double dist = manhattan(nx, ny, tx, ty);
            if (dist < minDist) {
                minDist = dist;
                best = d;
            }
        }
        return best;
    }

    private TWDirection bfsStep(int tx, int ty, int maxDepth) {
        int x0 = getX(), y0 = getY();
        if (x0 == tx && y0 == ty) return null;
        Queue<Node> q = new LinkedList<>();
        Set<Long> vis = new HashSet<>();
        q.add(new Node(x0, y0, null, 0, 0));
        vis.add(key(x0, y0));

        while (!q.isEmpty()) {
            Node n = q.poll();
            if (n.g >= maxDepth) continue;
            for (TWDirection d : TWDirection.values()) {
                int nx = n.x + d.dx;
                int ny = n.y + d.dy;
                if (isOutOfBounds(nx, ny) || isObstacle(nx, ny)) continue;
                if (vis.contains(key(nx, ny))) continue;
                if (nx == tx && ny == ty) return reconstructFirstDir(new Node(nx, ny, n, 0, 0));
                vis.add(key(nx, ny));
                q.add(new Node(nx, ny, n, n.g + 1, 0));
            }
        }
        return null;
    }

    // 工具
    private boolean isHole(int x, int y) {
        return getEnvironment().getObjectGrid().get(x, y) instanceof TWHole;
    }

    private boolean isRecentlyVisited(int x, int y) {
        for (int[] p : recentPositions) if (p[0] == x && p[1] == y) return true;
        return false;
    }

    private boolean isAdjacentToWallOrHole(int x, int y) {
        for (TWDirection d : TWDirection.values()) {
            int nx = x + d.dx;
            int ny = y + d.dy;
            if (isOutOfBounds(nx, ny) || isHole(nx, ny)) return true;
        }
        return false;
    }

    private TWDirection getRandomSafeDir() {
        List<TWDirection> s = new ArrayList<>();
        for (TWDirection d : TWDirection.values()) {
            int nx = getX() + d.dx;
            int ny = getY() + d.dy;
            if (!isOutOfBounds(nx, ny) && !isHole(nx, ny) && !isObstacle(nx, ny)) s.add(d);
        }
        return s.isEmpty() ? TWDirection.N : s.get(rand.nextInt(s.size()));
    }

    private TWThought safeMove() {
        return new TWThought(TWAction.MOVE, getRandomSafeDir());
    }

    private double manhattan(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    private long key(int x, int y) {
        return (long) x * 1000 + y;
    }

    private TWDirection reconstructFirstDir(Node n) {
        while (n.parent != null && n.parent.parent != null) n = n.parent;
        if (n.parent == null) return null;
        int dx = n.x - n.parent.x;
        int dy = n.y - n.parent.y;
        for (TWDirection d : TWDirection.values()) {
            if (d.dx == dx && d.dy == dy) return d;
        }
        return null;
    }

    private boolean isOutOfBounds(int x, int y) {
        return x < 0 || y < 0 || x >= getEnvironment().getxDimension() || y >= getEnvironment().getyDimension();
    }

    // 内部节点类
    private static class Node {
        int x, y;
        Node parent;
        double g, f;
        Node(int x, int y, Node p, double g, double h) {
            this.x = x; this.y = y; parent = p; this.g = g; f = g + h;
        }
    }

    // 环境实时校验
    // 校验指定位置是否存在可拾取的Tile
    private boolean isTileExist(int x, int y) {
        if (isOutOfBounds(x, y)) return false;
        Object obj = getEnvironment().getObjectGrid().get(x, y);
        return obj instanceof TWTile;
    }

    // 校验指定位置是否存在可填补的Hole
    private boolean isHoleExist(int x, int y) {
        if (isOutOfBounds(x, y)) return false;
        Object obj = getEnvironment().getObjectGrid().get(x, y);
        return obj instanceof TWHole;
    }

    private boolean isObstacle(int x, int y) {
        ObjectGrid2D grid = getEnvironment().getObjectGrid();
        if (isOutOfBounds(x, y)) return true;
        Object obj = grid.get(x, y);
        // 墙体/不可通行的障碍物（根据实际环境定义补充，例如墙、其他agent等）
        // 注意：需排除可交互的tile/hole/fuelStation
        return obj != null && !(obj instanceof TWTile || obj instanceof TWHole || obj instanceof TWFuelStation);
    }

    private boolean isReachableInZone(int tx, int ty, ZoneAssignment zone) {
        int x0 = getX();
        int y0 = getY();
        if (x0 == tx && y0 == ty) {
            return true;
        }

        Queue<Int2D> queue = new LinkedList<>();
        Set<Long> visited = new HashSet<>();
        queue.add(new Int2D(x0, y0));
        visited.add(key(x0, y0));

        while (!queue.isEmpty()) {
            Int2D current = queue.poll();
            for (TWDirection d : TWDirection.values()) {
                int nx = current.x + d.dx;
                int ny = current.y + d.dy;
                if (nx < zone.x1 || ny < zone.y1 || nx >= zone.x2 || ny >= zone.y2) {
                    continue;
                }
                if (isOutOfBounds(nx, ny) || isObstacle(nx, ny)) {
                    continue;
                }
                long key = key(nx, ny);
                if (visited.contains(key)) {
                    continue;
                }
                if (nx == tx && ny == ty) {
                    return true;
                }
                visited.add(key);
                queue.add(new Int2D(nx, ny));
            }
        }
        return false;
    }

    private int[] resolveBlockedCenterTarget(int tx, int ty, ZoneAssignment zone) {
        int[][] offsets = {{-1, 0}, {0, -1}, {1, 0}};
        for (int[] offset : offsets) {
            int nx = tx + offset[0];
            int ny = ty + offset[1];
            if (nx < zone.x1 || ny < zone.y1 || nx >= zone.x2 || ny >= zone.y2) {
                continue;
            }
            if (isOutOfBounds(nx, ny) || isObstacle(nx, ny) || isHole(nx, ny)) {
                continue;
            }
            if (isReachableInZone(nx, ny, zone)) {
                return new int[]{nx, ny};
            }
        }
        return null;
    }

    private void removeRecordAndSync(MemoryObjectType type, int x, int y) {
        this.memory.removeAgentPercept(x, y);
        this.memory.getMemoryGrid().set(x, y, null);
    }

    //锁（from AgentHanny）
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
    private static class TargetLease {
        private final String owner;
        private final double priority;
        private final long step;
        private final int ttlSteps;
        private final long expiresAtStep;

        private TargetLease(String owner, double priority, long step, int ttlSteps) {
            this.owner = owner;
            this.priority = priority;
            this.step = step;
            this.ttlSteps = Math.max(1, ttlSteps);
            this.expiresAtStep = this.step + this.ttlSteps;
        }
    }

    private void handleTeammateTargetLock(ParsedMessage parsed) {
        Double priority = parseTargetLockPriority(parsed.payload);
        if (priority == null) {
            return;
        }
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

    private void handleTeammateTargetRelease(ParsedMessage parsed) {
        String key = MemorySideCard.cellKey(parsed.x, parsed.y);
        TargetLease existing = teammateTargetLeases.get(key);
        if (existing == null) {
            return;
        }
        if (existing.owner.equals(parsed.from)) {
            teammateTargetLeases.remove(key);
        }
    }

    private boolean hasActiveTargetLock() {
        return lockedTargetCell != null && lockedTargetType != null;
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

    private void ensureActiveLockStillValid(long step) {
        if (!hasActiveTargetLock()) {
            return;
        }

        if (step > lockedTargetExpiryStep) {
            releaseOwnTargetLock("local_lease_expired");
            return;
        }

        // Priority arbitration: if teammate holds a stronger lock on the same cell, release.
        TargetLease teammate = teammateTargetLeases.get(MemorySideCard.cellKey(lockedTargetCell.x, lockedTargetCell.y));
        if (teammate != null) {
            if (!isPreferredLock(getName(), lockedTargetPriority, lockedTargetStep, teammate.owner, teammate.priority, teammate.step)) {
                releaseOwnTargetLock("lost_priority");
                return;
            }
        }

        // If current sensor already proves mismatch/disappearance, release immediately.
        if (!isInSensorRange(lockedTargetCell.x, lockedTargetCell.y)) {
            return;
        }

        TWEntity obj = (TWEntity) this.getEnvironment().getObjectGrid().get(lockedTargetCell.x, lockedTargetCell.y);
        MemoryObjectType observed = MemoryObjectType.fromEntity(obj);
        if (observed != lockedTargetType) {
            releaseOwnTargetLock("target_changed_or_missing");
            return;
        }
    }

    private boolean isInSensorRange(int x, int y) {
        int dx = Math.abs(x - this.getX());
        int dy = Math.abs(y - this.getY());
        return Math.max(dx, dy) <= Parameters.defaultSensorRange;
    }

    private boolean isOwnLockAt(MemoryObjectType type, int x, int y) {
        return hasActiveTargetLock()
                && lockedTargetType == type
                && lockedTargetCell.x == x
                && lockedTargetCell.y == y;
    }

    private boolean isBlockedByPreferredTeammateLock(int x, int y, double myPriority, long myStep) {
        TargetLease teammate = teammateTargetLeases.get(MemorySideCard.cellKey(x, y));
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

        // Mix to a stable pseudo-random [0,1) value.
        seed ^= (seed >>> 33);
        seed *= 0xff51afd7ed558ccdL;
        seed ^= (seed >>> 33);
        seed *= 0xc4ceb9fe1a85ec53L;
        seed ^= (seed >>> 33);

        long positive = seed & Long.MAX_VALUE;
        return (positive % 1000000L) / 1000000.0;
    }

    private boolean tryAcquireOrKeepTargetLock(MemorySideCardEntry entry, long step) {
        if (entry == null) {
            return false;
        }

        int x = entry.getX();
        int y = entry.getY();
        MemoryObjectType type = entry.getType();

        if (isOwnLockAt(type, x, y)) {
            return true;
        }

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
                CommType.TARGET_LOCK,
                x,
                y,
                encodeTargetLockPayload(priority, ownTargetLockTtlSteps));
        return true;
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
        lockedTargetType = null;
        lockedTargetPriority = 0.0;
        lockedTargetStep = -1L;
        lockedTargetExpiryStep = -1L;
    }



    private void processIncomingMessages(long step) {
        List<Message> inbox = new ArrayList<Message>(this.getEnvironment().getMessages());
        for (Message message : inbox) {
            ParsedMessage parsed = parseProtocolMessage(message);
            if (parsed == null) {
                continue;
            }

            if (parsed.from == null || parsed.from.equals(this.getName())) {
                continue;
            }
            if (!isMessageForMe(parsed)) {
                continue;
            }

            switch (parsed.type) {
                case OBS_NEW_TILE:
                case OBS_NEW_HOLE:
                case OBS_OBSTACLE:
                    break;
                case OBS_FUEL_ONCE:
                    boolean exists = false;
                    for (int[] p : knownFuelStations) {
                        if (p[0] == parsed.x && p[1] == parsed.y) {
                            exists = true;
                            break;
                        }
                    }
                    if (!exists) {
                        knownFuelStations.add(new int[]{parsed.x, parsed.y});
                        //outbox.add(encodeProtocolMessage(CommType.OBS_FUEL_ONCE, fx, fy, ""));
                    }
                    break;
                case OBS_SENSOR_SNAPSHOT:
                    //processSnapshotMessage(parsed);
                    break;
                case TARGET_LOCK:
                    handleTeammateTargetLock(parsed);
                    break;
                case TARGET_RELEASE:
                    handleTeammateTargetRelease(parsed);
                    break;
                case ACTION_PICKUP_TILE:
                    removeRecordAndSync(MemoryObjectType.TILE, parsed.x, parsed.y);
                    break;
                case ACTION_FILL_HOLE:
                    removeRecordAndSync(MemoryObjectType.HOLE, parsed.x, parsed.y);
                    break;
                default:
                    break;
            }
        }
    }
}