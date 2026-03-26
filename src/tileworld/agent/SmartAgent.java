package tileworld.agent;

import java.util.*;
import sim.field.grid.ObjectGrid2D;
import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.*;
import tileworld.exceptions.CellBlockedException;


public class SmartAgent extends TWAgent {

    // ===================== 配置 =====================
    private static final double FUEL_FORCE_REFuel = 150;   // 强制加油
    private static final double FUEL_WARNING = 200;       // 预警层
    private static final int RECENT_MAX = 10;
    private static final double RANDOM_STEP = 0.05;
    private static final int LOCAL_RANGE = 12;
    private static final int ASTAR_MAX_NODES = 150;
    private static final int SAFETY_FUEL_BUFFER = 20;     // 往返安全余量

    // ===================== 状态 =====================
    private final Deque<int[]> recentPositions = new ArrayDeque<>();
    private final Set<int[]> knownFuelStations = new HashSet<>(); // 记忆加油站
    private final Random rand = new Random();

    // 网格搜索状态
    private boolean gridSearchInitialized = false;
    private List<Integer> gridCenterXs = new ArrayList<>();
    private List<Integer> gridCenterYs = new ArrayList<>();
    private int currentGridIndex = 0;

    // ===================== 构造 =====================
    public SmartAgent(int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
    }

    // ===================== 必须覆写的4个方法 =====================
    @Override
    public String getName() {
        return "SmartAgent-" + this.hashCode();
    }

    @Override
    public void communicate() { }

    @Override
    protected TWThought think() {
        sense();
        int x = getX();
        int y = getY();
        double fuel = getFuelLevel();

        // 记录最近走过的位置
        recentPositions.offer(new int[]{x, y});
        if (recentPositions.size() > RECENT_MAX) recentPositions.poll();

        // 记忆当前视野内的加油站（新增）
        memorizeVisibleFuelStations();

        // ===================== 已知加油站 =====================
        if (!knownFuelStations.isEmpty()) {

            // ===================== 1. 强制加油（<150） =====================
            if (fuel < FUEL_FORCE_REFuel) {
                if (this.getEnvironment().inFuelStation(this)) {
                    System.out.println(String.format("%s 已在燃料站位置，执行加油", getName()));
                    return new TWThought(TWAction.REFUEL, TWDirection.Z);
                }
                return thinkRefuel();
            }

            // ===================== 2. 预警模式（<200）：只做近距离任务 =====================
            // ===================== 3. 正常模式（>=200） =====================
            boolean warningMode = fuel < FUEL_WARNING;

            // 有瓷砖/洞 → 拾取/填补
            boolean hasTile = memory.getClosestObjectInSensorRange(TWTile.class) != null;
            boolean hasHole = memory.getClosestObjectInSensorRange(TWHole.class) != null;
            if (hasTile || hasHole) {
                return thinkPickFill(warningMode);
            }

            // 视野有加油站 → 加油
            boolean hasFuel = memory.getClosestObjectInSensorRange(TWFuelStation.class) != null;
            if (hasFuel) {
                return thinkRefuel();
            }

            // 探索（预警模式下缩小范围）
            return thinkExplore(warningMode);
        } else {
            
            return thinkGridSearch();
        }

        
    }

    @Override
    protected void act(TWThought thought) {
        if (thought == null) thought = safeMove();

        try {
            switch (thought.getAction()) {
                case MOVE:
                    move(thought.getDirection());
                    break;
                case PICKUP:
                    TWTile t = (TWTile) getEnvironment().getObjectGrid().get(getX(), getY());
                    if (t != null && carriedTiles.size() < 3) pickUpTile(t);
                    break;
                case PUTDOWN:
                    TWHole h = (TWHole) getEnvironment().getObjectGrid().get(getX(), getY());
                    if (h != null && !carriedTiles.isEmpty()) putTileInHole(h);
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
        final int HALF_BLOCK = 3;

        // 1. 初始化：生成蛇形往返的所有网格中心点（正行→反行→正行）
        if (!gridSearchInitialized) {
            gridCenterXs.clear();
            gridCenterYs.clear();
            currentGridIndex = 0;

            int mapW = getEnvironment().getxDimension();
            int mapH = getEnvironment().getyDimension();
            boolean reverse = false;

            // 逐行生成，蛇形往返
            for (int blockY = 0; blockY < mapH; blockY += BLOCK_SIZE) {
                List<Integer> rowCentersX = new ArrayList<>();
                int centerY = blockY + HALF_BLOCK;
                centerY = Math.min(centerY, mapH - 1);

                // 生成当前行所有中心点X
                for (int blockX = 0; blockX < mapW; blockX += BLOCK_SIZE) {
                    int centerX = blockX + HALF_BLOCK;
                    centerX = Math.min(centerX, mapW - 1);
                    rowCentersX.add(centerX);
                }

                // 蛇形核心：偶数行正序，奇数行反序
                if (reverse) {
                    Collections.reverse(rowCentersX);
                }
                reverse = !reverse;

                // 加入总列表
                for (int x : rowCentersX) {
                    gridCenterXs.add(x);
                    gridCenterYs.add(centerY);
                }
            }

            gridSearchInitialized = true;
        }

        // 2. 找到加油站 → 停止搜索
        if (!knownFuelStations.isEmpty()) {
            gridSearchInitialized = false;
            currentGridIndex = 0;
            return think();
        }

        // 3. 全部扫描完毕 → 重置
        if (currentGridIndex >= gridCenterXs.size()) {
            gridSearchInitialized = false;
            currentGridIndex = 0;
            return safeMove();
        }

        // 4. 前往下一个蛇形中心点
        int targetX = gridCenterXs.get(currentGridIndex);
        int targetY = gridCenterYs.get(currentGridIndex);

        // 已到达 → 扫描视野，然后下一个点
        if (getX() == targetX && getY() == targetY) {
            memorizeVisibleFuelStations();
            currentGridIndex++;
        }

        // 移动
        return hierarchicalPlan(targetX, targetY);
    }

    // ===================== 记忆加油站 =====================
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
                    }
                }
            }
        }
    }

    // ===================== 获取最近的记忆加油站 =====================
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

    // ===================== 往返油量检查（新增核心） =====================
    private boolean hasEnoughFuelForRoundTrip(int tx, int ty) {
        int[] fuelStation = getClosestKnownFuelStation();
        if (fuelStation == null) return true; // 无加油站 → 允许

        int x = getX(), y = getY();
        double toTarget = Math.abs(x - tx) + Math.abs(y - ty);
        double targetToFuel = Math.abs(tx - fuelStation[0]) + Math.abs(ty - fuelStation[1]);
        double total = toTarget + targetToFuel + SAFETY_FUEL_BUFFER;

        return getFuelLevel() >= total;
    }

    // ===================== 加油模式（使用记忆） =====================
    private TWThought thinkRefuel() {
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

    // ===================== 捡瓷砖/补洞（预警+油量检查） =====================
    private TWThought thinkPickFill(boolean warningMode) {
        int x = getX(), y = getY();

        // 有瓷砖 → 补洞
        if (!carriedTiles.isEmpty()) {
            TWHole h = (TWHole) memory.getClosestObjectInSensorRange(TWHole.class);
            if (h != null) {
                int hx = h.getX(), hy = h.getY();

                // 预警模式：只去近的
                if (warningMode && (Math.abs(hx - x) + Math.abs(hy - y) > 15)) {
                    return thinkRefuel();
                }

                // 油量够往返才去
                if (hasEnoughFuelForRoundTrip(hx, hy)) {
                    return hierarchicalPlan(hx, hy);
                } else {
                    return thinkRefuel();
                }
            }
        }

        // 没瓷砖 → 捡瓷砖
        TWTile t = (TWTile) memory.getClosestObjectInSensorRange(TWTile.class);
        if (t != null) {
            int tx = t.getX(), ty = t.getY();

            if (warningMode && (Math.abs(tx - x) + Math.abs(ty - y) > 15)) {
                return thinkRefuel();
            }

            if (hasEnoughFuelForRoundTrip(tx, ty)) {
                return hierarchicalPlan(tx, ty);
            } else {
                return thinkRefuel();
            }
        }

        return safeMove();
    }

    // ===================== 探索（预警模式缩小范围） =====================
    private TWThought thinkExplore(boolean warningMode) {
        int x = getX(), y = getY();

        if (rand.nextDouble() < RANDOM_STEP) {
            return new TWThought(TWAction.MOVE, getRandomSafeDir());
        }

        List<TWDirection> valid = new ArrayList<>();
        for (TWDirection d : TWDirection.values()) {
            int nx = x + d.dx;
            int ny = y + d.dy;
            if (isOutOfBounds(nx, ny) || isHole(nx, ny)) continue;
            if (isRecentlyVisited(nx, ny)) continue;

            // 预警模式：不远离加油站
            if (warningMode) {
                int[] fs = getClosestKnownFuelStation();
                if (fs != null) {
                    double oldDist = Math.abs(x - fs[0]) + Math.abs(y - fs[1]);
                    double newDist = Math.abs(nx - fs[0]) + Math.abs(ny - fs[1]);
                    if (newDist > oldDist) continue;
                }
            }

            valid.add(d);
        }

        // 壁沿优先
        for (TWDirection d : valid) {
            int nx = x + d.dx;
            int ny = y + d.dy;
            if (isAdjacentToWallOrHole(nx, ny)) {
                return new TWThought(TWAction.MOVE, d);
            }
        }

        return valid.isEmpty() ? safeMove() : new TWThought(TWAction.MOVE, valid.get(0));
    }

    // ===================== 路径规划 =====================
    private TWThought hierarchicalPlan(int tx, int ty) {
        int x = getX(), y = getY();
        if (x == tx && y == ty) {
            if (isHole(x, y) && !carriedTiles.isEmpty())
                return new TWThought(TWAction.PUTDOWN, null);
            if (!isHole(x, y) && carriedTiles.size() < 3)
                return new TWThought(TWAction.PICKUP, null);
            return safeMove();
        }

        TWDirection d = limitedAStar(tx, ty);
        if (d != null) return new TWThought(TWAction.MOVE, d);

        d = greedyDir(tx, ty);
        if (d != null) return new TWThought(TWAction.MOVE, d);

        d = bfsStep(tx, ty, 2);
        if (d != null) return new TWThought(TWAction.MOVE, d);

        return safeMove();
    }

    // ===================== A* / 贪心 / BFS =====================
    private TWDirection limitedAStar(int tx, int ty) {
        int x0 = getX(), y0 = getY();
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
                if (isOutOfBounds(nx, ny) || isHole(nx, ny)) continue;
                if (closed.contains(key(nx, ny))) continue;
                if (Math.abs(nx - x0) > LOCAL_RANGE || Math.abs(ny - y0) > LOCAL_RANGE) continue;
                open.add(new Node(nx, ny, cur, cur.g + 1, manhattan(nx, ny, tx, ty)));
            }
        }
        return null;
    }

    private TWDirection greedyDir(int tx, int ty) {
        TWDirection best = null;
        double minDist = 9999;
        for (TWDirection d : TWDirection.values()) {
            int nx = getX() + d.dx;
            int ny = getY() + d.dy;
            if (isOutOfBounds(nx, ny) || isHole(nx, ny)) continue;
            double dist = manhattan(nx, ny, tx, ty);
            if (dist < minDist) {
                minDist = dist;
                best = d;
            }
        }
        return best;
    }

    private TWDirection bfsStep(int tx, int ty, int maxDepth) {
        Queue<Node> q = new LinkedList<>();
        Set<Long> vis = new HashSet<>();
        int x0 = getX(), y0 = getY();
        q.add(new Node(x0, y0, null, 0, 0));
        vis.add(key(x0, y0));

        while (!q.isEmpty()) {
            Node n = q.poll();
            if (n.g >= maxDepth) continue;
            for (TWDirection d : TWDirection.values()) {
                int nx = n.x + d.dx;
                int ny = n.y + d.dy;
                if (isOutOfBounds(nx, ny) || isHole(nx, ny)) continue;
                if (vis.contains(key(nx, ny))) continue;
                if (nx == tx && ny == ty) return reconstructFirstDir(new Node(nx, ny, n, 0, 0));
                vis.add(key(nx, ny));
                q.add(new Node(nx, ny, n, n.g + 1, 0));
            }
        }
        return null;
    }

    // ===================== 工具 =====================
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
            if (!isOutOfBounds(nx, ny) && !isHole(nx, ny)) s.add(d);
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

    // ===================== 内部节点类 =====================
    private static class Node {
        int x, y;
        Node parent;
        double g, f;
        Node(int x, int y, Node p, double g, double h) {
            this.x = x; this.y = y; parent = p; this.g = g; f = g + h;
        }
    }
}