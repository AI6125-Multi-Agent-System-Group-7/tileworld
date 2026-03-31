package tileworld.agent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

import sim.field.grid.ObjectGrid2D;
import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;

/**
 * AgentDeng — 效用驱动 + 事件重规划 + 未覆盖区域探索
 *
 * 修复版：
 *   - 处理脚下对象无法操作时不卡住（背包满 / 没 Tile）
 *   - 到达目标发现对象消失时清理记忆并立即 replan
 *   - 排除距离=0 但无法操作的候选目标
 *   - 记忆中过期/被抢的对象主动清理
 */
public class AgentDeng extends Group7AgentBase {

    // ============ 参数 ============
    private static final int FUEL_SAFETY_MARGIN = 15;

    private static final double UTILITY_REFUEL   = 200.0;
    private static final double UTILITY_PUTDOWN  = 50.0;
    private static final double UTILITY_PICKUP   = 30.0;
    private static final double UTILITY_EXPLORE  = 8.0;

    // ============ 状态 ============
    private boolean[][] visited;
    private Int2D bestTarget;
    private TWEntity bestTargetEntity;
    private int stepCount;

    // 卡死检测
    private Int2D lastPos;
    private int stuckCounter;
    private static final int STUCK_THRESHOLD = 5;

    // 寻路失败黑名单（避免反复选同一个不可达目标）
    private Set<String> pathFailBlacklist = new HashSet<String>();
    private int blacklistClearCountdown = 0;
    private static final int BLACKLIST_CLEAR_INTERVAL = 20;

    // 队友目标锁定追踪（协同去冲突）
    private static final int TARGET_LOCK_TTL = 15; // 自身锁定 TTL（步数）
    private Int2D lockedTargetCell = null;          // 自身当前锁定的目标位置
    private long lockedTargetStep = -1;             // 锁定时间
    private long lockedTargetExpiry = -1;           // 锁定过期时间

    /** 队友锁定的目标: key = "x,y", value = 过期步数 */
    private Map<String, Long> teammateLockedTargets = new HashMap<String, Long>();

    public AgentDeng(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
        this.visited = new boolean[env.getxDimension()][env.getyDimension()];
    }

    // ================================================================
    //  THINK
    // ================================================================
    @Override
    protected TWThought think() {
        rememberFuelStationsInSensorRange();
        processIncomingMessages();
        markVisited();
        cleanStaleMemory();
        updateStuckDetection();

        // 定期清理寻路黑名单
        if (blacklistClearCountdown > 0) {
            blacklistClearCountdown--;
        } else {
            pathFailBlacklist.clear();
            blacklistClearCountdown = BLACKLIST_CLEAR_INTERVAL;
        }

        // ---- 卡死应急：连续多步没移动 → 清除所有记忆中的障碍物 ----
        if (stuckCounter >= STUCK_THRESHOLD) {
            clearObstacleMemory();
            clearAllTargets();
            pathFailBlacklist.clear();
            stuckCounter = 0;
        }

        // ---- 1. 脚下免费操作 ----
        TWEntity here = (TWEntity) this.getEnvironment().getObjectGrid().get(getX(), getY());
        if (here instanceof TWHole && hasTile()) {
            clearAllTargets();
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (here instanceof TWTile && carriedTiles.size() < 3) {
            clearAllTargets();
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }
        if (this.getEnvironment().inFuelStation(this)
                && this.getFuelLevel() < FUEL_SAFETY_MARGIN + REFUEL_MARGIN_COMFORTABLE) {
            clearAllTargets();
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }

        // ---- 2. 到达目标位置后的处理 ----
        if (bestTarget != null && at(bestTarget)) {
            if (bestTargetEntity != null && targetGoneAtLocation(bestTargetEntity)) {
                this.memory.removeAgentPercept(bestTargetEntity.getX(), bestTargetEntity.getY());
                this.memory.getMemoryGrid().set(bestTargetEntity.getX(), bestTargetEntity.getY(), null);
            }
            clearAllTargets();
        }

        // ---- 3. 判断是否需要 replan ----
        boolean needReplan = false;
        if (bestTarget == null) {
            needReplan = true;
        } else if (bestTargetEntity != null && targetGoneInSight(bestTargetEntity)) {
            this.memory.removeAgentPercept(bestTargetEntity.getX(), bestTargetEntity.getY());
            this.memory.getMemoryGrid().set(bestTargetEntity.getX(), bestTargetEntity.getY(), null);
            needReplan = true;
        } else if (currentPath == null || currentPath.isEmpty()) {
            needReplan = true;
        } else if (hasBetterCandidate()) {
            needReplan = true;
        }

        // ---- 4. 选最优目标 ----
        if (needReplan) {
            selectBestTarget();
        }

        if (bestTarget == null) {
            return randomStep();
        }

        // ---- 5. 防卡死：target 就是自己的位置 → 不要 stepToward，直接 fallback ----
        if (at(bestTarget)) {
            clearAllTargets();
            return randomStep();
        }

        TWThought result = stepToward(bestTarget);

        // ---- 6. 防卡死：planPath 失败 → 加入黑名单，贪心走一步 or 随机 ----
        if (result.getAction() == TWAction.MOVE && result.getDirection() == TWDirection.Z) {
            // 保存目标再清除（修复：之前 clearAllTargets 后 bestTarget 变 null）
            Int2D failedTarget = bestTarget;
            if (failedTarget != null) {
                pathFailBlacklist.add(failedTarget.x + "," + failedTarget.y);
            }
            clearAllTargets();
            // 用保存的目标做贪心
            TWThought greedy = greedyStepToward(failedTarget);
            if (greedy != null) return greedy;
            return randomStep();
        }

        return result;
    }

    // ================================================================
    //  目标选择
    // ================================================================

    private static class Candidate {
        final Int2D pos;
        final TWEntity entity;
        final double utility;

        Candidate(Int2D pos, TWEntity entity, double utility) {
            this.pos = pos;
            this.entity = entity;
            this.utility = utility;
        }
    }

    private void selectBestTarget() {
        clearAllTargets();

        List<Candidate> candidates = new ArrayList<Candidate>();

        // 候选 1：加油站（不受 fuelSafe 约束，任何时候都可以去）
        TWFuelStation station = findFuelStationInMemory();
        if (station != null) {
            double u = utilityRefuel(station);
            if (u > 0) {
                candidates.add(new Candidate(
                        new Int2D(station.getX(), station.getY()), station, u));
            }
        }

        // 候选 2：Hole（需要手上有 Tile）
        if (hasTile()) {
            collectHoleCandidates(candidates);
        }

        // 候选 3：Tile（需要没满）
        if (carriedTiles.size() < 3) {
            collectTileCandidates(candidates);
        }

        // 候选 4：探索目标
        Int2D exploreTarget = pickExploreTarget();
        if (exploreTarget != null) {
            double u = utilityExplore(exploreTarget);
            if (u > 0) {
                candidates.add(new Candidate(exploreTarget, null, u));
            }
        }

        Candidate best = null;
        for (Candidate c : candidates) {
            // 跳过寻路失败的黑名单目标（避免反复选同一个不可达目标）
            if (pathFailBlacklist.contains(c.pos.x + "," + c.pos.y)) continue;
            // 跳过队友已锁定的目标（协同去冲突），紧急加油除外
            if (c.utility < 9999 && isLockedByTeammate(c.pos.x, c.pos.y)) continue;
            if (best == null || c.utility > best.utility) {
                best = c;
            }
        }

        if (best != null) {
            bestTarget = best.pos;
            bestTargetEntity = best.entity;
            acquireOwnTargetLock(best.pos, best.utility);
        }

        // 【关键修复】所有候选都被排除了 → 燃料紧张时强制去加油站
        if (bestTarget == null && station != null) {
            bestTarget = new Int2D(station.getX(), station.getY());
            bestTargetEntity = station;
        }
    }

    // ================================================================
    //  效用函数
    // ================================================================

    /** 去掉回程油费后还能走多少步 */
    private static final int REFUEL_MARGIN_COMFORTABLE = 60;

    private double utilityRefuel(TWFuelStation station) {
        int dist = manhattan(getX(), getY(), station.getX(), station.getY());
        // 燃料余量 = 当前油量 - 回站路程 - 安全边际
        double margin = this.getFuelLevel() - dist - FUEL_SAFETY_MARGIN;

        // 紧急：余量不够回站
        if (margin <= 0) {
            return 9999;
        }

        // 余量充足，完全不考虑加油
        if (margin > REFUEL_MARGIN_COMFORTABLE) {
            return -1;
        }

        // 余量 0~COMFORTABLE 之间：指数级增长的紧迫感
        // margin=0 → urgency=1, margin=COMFORTABLE → urgency≈0
        double normalized = margin / REFUEL_MARGIN_COMFORTABLE; // 0~1
        double urgency = Math.exp(-4.0 * normalized);           // e^0=1 → e^-4≈0.018
        return UTILITY_REFUEL * urgency / (1.0 + dist);
    }

    private double utilityHole(TWHole hole) {
        int dist = manhattan(getX(), getY(), hole.getX(), hole.getY());

        // 【修复】距离=0 但脚下没 Hole 实体 → 已消失，跳过
        if (dist == 0) {
            Object actual = this.getEnvironment().getObjectGrid().get(getX(), getY());
            if (!(actual instanceof TWHole)) return -1;
        }

        if (!fuelSafeToVisit(hole.getX(), hole.getY(), dist)) return -1;

        double timeLeft = hole.getTimeLeft(this.getEnvironment().schedule.getTime());
        if (timeLeft <= dist) return -1;

        double freshness = timeLeft / (timeLeft + dist);
        return UTILITY_PUTDOWN * freshness / (1.0 + dist);
    }

    private double utilityTile(TWTile tile) {
        int dist = manhattan(getX(), getY(), tile.getX(), tile.getY());

        // 【修复】距离=0 → 脚下处理（如果背包满了就跳过，不要卡住）
        if (dist == 0) {
            if (carriedTiles.size() >= 3) return -1;
            Object actual = this.getEnvironment().getObjectGrid().get(getX(), getY());
            if (!(actual instanceof TWTile)) return -1;
        }

        if (!fuelSafeToVisit(tile.getX(), tile.getY(), dist)) return -1;

        double timeLeft = tile.getTimeLeft(this.getEnvironment().schedule.getTime());
        if (timeLeft <= dist) return -1;

        double freshness = timeLeft / (timeLeft + dist);
        double base = UTILITY_PICKUP;
        if (nearestKnownHole() != null) {
            base += 10;
        }
        return base * freshness / (1.0 + dist);
    }

    private double utilityExplore(Int2D pos) {
        int dist = manhattan(getX(), getY(), pos.x, pos.y);
        if (dist == 0) return -1; // 已经在这了
        if (!fuelSafeToVisit(pos.x, pos.y, dist)) return -1;
        return UTILITY_EXPLORE / (1.0 + dist);
    }

    // ---- 候选收集 ----

    private void collectHoleCandidates(List<Candidate> candidates) {
        ObjectGrid2D memGrid = this.memory.getMemoryGrid();
        int xDim = this.getEnvironment().getxDimension();
        int yDim = this.getEnvironment().getyDimension();
        for (int x = 0; x < xDim; x++) {
            for (int y = 0; y < yDim; y++) {
                Object obj = memGrid.get(x, y);
                if (obj instanceof TWHole) {
                    double u = utilityHole((TWHole) obj);
                    if (u > 0) {
                        candidates.add(new Candidate(new Int2D(x, y), (TWHole) obj, u));
                    }
                }
            }
        }
    }

    private void collectTileCandidates(List<Candidate> candidates) {
        ObjectGrid2D memGrid = this.memory.getMemoryGrid();
        int xDim = this.getEnvironment().getxDimension();
        int yDim = this.getEnvironment().getyDimension();
        for (int x = 0; x < xDim; x++) {
            for (int y = 0; y < yDim; y++) {
                Object obj = memGrid.get(x, y);
                if (obj instanceof TWTile) {
                    double u = utilityTile((TWTile) obj);
                    if (u > 0) {
                        candidates.add(new Candidate(new Int2D(x, y), (TWTile) obj, u));
                    }
                }
            }
        }
    }

    // ================================================================
    //  燃料安全
    // ================================================================

    private boolean fuelSafeToVisit(int tx, int ty, int distToTarget) {
        TWFuelStation station = findFuelStationInMemory();
        if (station == null) {
            return this.getFuelLevel() > distToTarget + FUEL_SAFETY_MARGIN * 3;
        }
        int distTargetToStation = manhattan(tx, ty, station.getX(), station.getY());
        return this.getFuelLevel() > distToTarget + distTargetToStation + FUEL_SAFETY_MARGIN;
    }

    // ================================================================
    //  记忆清理：清除传感器范围内已消失的对象
    // ================================================================

    private void cleanStaleMemory() {
        int r = Parameters.defaultSensorRange;
        ObjectGrid2D envGrid = this.getEnvironment().getObjectGrid();
        ObjectGrid2D memGrid = this.memory.getMemoryGrid();

        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int x = getX() + dx;
                int y = getY() + dy;
                if (!this.getEnvironment().isInBounds(x, y)) continue;

                Object memObj = memGrid.get(x, y);
                if (memObj == null) continue;
                if (memObj instanceof TWFuelStation) continue; // 加油站永不消失

                // 传感器能看到这个位置 — 检查实际环境
                Object envObj = envGrid.get(x, y);

                // 记忆说有 Tile 但实际没了（被其他 Agent 捡走或过期）
                if (memObj instanceof TWTile && !(envObj instanceof TWTile)) {
                    memGrid.set(x, y, null);
                    this.memory.removeAgentPercept(x, y);
                }
                // 记忆说有 Hole 但实际没了（被填或过期）
                if (memObj instanceof TWHole && !(envObj instanceof TWHole)) {
                    memGrid.set(x, y, null);
                    this.memory.removeAgentPercept(x, y);
                }
                // 记忆说有障碍物但实际没了（过期消失）
                if (memObj instanceof TWObstacle && !(envObj instanceof TWObstacle)) {
                    memGrid.set(x, y, null);
                    this.memory.removeAgentPercept(x, y);
                }
            }
        }
    }

    // ================================================================
    //  探索
    // ================================================================

    private void markVisited() {
        int r = Parameters.defaultSensorRange;
        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int x = getX() + dx;
                int y = getY() + dy;
                if (this.getEnvironment().isInBounds(x, y)) {
                    visited[x][y] = true;
                }
            }
        }
    }

    private Int2D pickExploreTarget() {
        int xDim = this.getEnvironment().getxDimension();
        int yDim = this.getEnvironment().getyDimension();

        Int2D best = null;
        int bestDist = Integer.MAX_VALUE;

        for (int x = 0; x < xDim; x++) {
            for (int y = 0; y < yDim; y++) {
                if (!visited[x][y] && !isBlocked(x, y)) {
                    int d = manhattan(getX(), getY(), x, y);
                    if (d > 0 && d < bestDist) {
                        bestDist = d;
                        best = new Int2D(x, y);
                    }
                }
            }
        }

        if (best == null) {
            resetVisited();
            // 全覆盖后随机选
            for (int i = 0; i < 50; i++) {
                int x = this.getEnvironment().random.nextInt(xDim);
                int y = this.getEnvironment().random.nextInt(yDim);
                int d = manhattan(getX(), getY(), x, y);
                if (d > 0 && this.getEnvironment().isInBounds(x, y) && !isBlocked(x, y)) {
                    return new Int2D(x, y);
                }
            }
        }

        return best;
    }

    private void resetVisited() {
        for (int x = 0; x < visited.length; x++) {
            for (int y = 0; y < visited[x].length; y++) {
                visited[x][y] = false;
            }
        }
    }

    // ================================================================
    //  Replan 辅助
    // ================================================================

    private boolean hasBetterCandidate() {
        if (bestTarget == null) return true;

        double currentUtility = currentTargetUtility();
        if (currentUtility <= 0) return true;

        // 视野内 Tile
        TWTile vTile = closestVisibleTile();
        if (vTile != null && carriedTiles.size() < 3) {
            double u = utilityTile(vTile);
            if (u > currentUtility * 1.3) return true;
        }

        // 视野内 Hole
        TWHole vHole = closestVisibleHole();
        if (vHole != null && hasTile()) {
            double u = utilityHole(vHole);
            if (u > currentUtility * 1.3) return true;
        }

        // 燃料紧急度变化 — 只有紧急情况（效用=9999）才打断当前任务
        TWFuelStation station = findFuelStationInMemory();
        if (station != null && !(bestTargetEntity instanceof TWFuelStation)) {
            double u = utilityRefuel(station);
            if (u >= 9999) return true;
        }

        return false;
    }

    private double currentTargetUtility() {
        if (bestTargetEntity instanceof TWHole) {
            return utilityHole((TWHole) bestTargetEntity);
        } else if (bestTargetEntity instanceof TWTile) {
            return utilityTile((TWTile) bestTargetEntity);
        } else if (bestTargetEntity instanceof TWFuelStation) {
            return utilityRefuel((TWFuelStation) bestTargetEntity);
        } else {
            return utilityExplore(bestTarget);
        }
    }

    /**
     * 目标在视野范围内，但实际已消失
     */
    private boolean targetGoneInSight(TWEntity target) {
        if (target == null || target instanceof TWFuelStation) return false;
        int dx = Math.abs(target.getX() - getX());
        int dy = Math.abs(target.getY() - getY());
        if (Math.max(dx, dy) > Parameters.defaultSensorRange) return false;

        Object actual = this.getEnvironment().getObjectGrid().get(target.getX(), target.getY());
        if (target instanceof TWTile) return !(actual instanceof TWTile);
        if (target instanceof TWHole) return !(actual instanceof TWHole);
        return false;
    }

    /**
     * 已到达目标位置，检查对象是否还在
     */
    private boolean targetGoneAtLocation(TWEntity target) {
        if (target == null || target instanceof TWFuelStation) return false;
        Object actual = this.getEnvironment().getObjectGrid().get(target.getX(), target.getY());
        if (target instanceof TWTile) return !(actual instanceof TWTile);
        if (target instanceof TWHole) return !(actual instanceof TWHole);
        return false;
    }

    // ================================================================
    //  工具方法
    // ================================================================

    private void clearAllTargets() {
        releaseOwnTargetLock("clear");
        clearPlan();
        bestTarget = null;
        bestTargetEntity = null;
    }

    private boolean at(Int2D pos) {
        return pos != null && getX() == pos.x && getY() == pos.y;
    }

    /** 卡死检测：记录位置，连续不动则递增计数 */
    private void updateStuckDetection() {
        Int2D currentPos = new Int2D(getX(), getY());
        if (lastPos != null && lastPos.equals(currentPos)) {
            stuckCounter++;
        } else {
            stuckCounter = 0;
        }
        lastPos = currentPos;
    }

    /**
     * 覆写 isBlocked：传感器范围内信任实际环境 + 记忆，
     * 范围外只信任实际环境（避免过期障碍物形成幽灵墙）。
     */
    @Override
    protected boolean isBlocked(int x, int y) {
        if (!this.getEnvironment().isInBounds(x, y)) return true;
        int dx = Math.abs(x - getX());
        int dy = Math.abs(y - getY());
        if (Math.max(dx, dy) <= Parameters.defaultSensorRange) {
            // 传感器范围内：用真实环境判断
            if (this.getEnvironment().isCellBlocked(x, y)) return true;
        }
        // 所有位置都参考记忆（范围外只靠记忆，不偷看环境）
        return this.memory.isCellBlocked(x, y);
    }

    /** 清除记忆中所有障碍物（应急用，解除幽灵墙） */
    private void clearObstacleMemory() {
        ObjectGrid2D memGrid = this.memory.getMemoryGrid();
        int xDim = this.getEnvironment().getxDimension();
        int yDim = this.getEnvironment().getyDimension();
        for (int x = 0; x < xDim; x++) {
            for (int y = 0; y < yDim; y++) {
                Object obj = memGrid.get(x, y);
                if (obj instanceof TWObstacle) {
                    memGrid.set(x, y, null);
                    this.memory.removeAgentPercept(x, y);
                }
            }
        }
    }

    private int manhattan(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    /**
     * 贪心走一步：不用 A*，直接选离 target 最近的相邻格子
     */
    private TWThought greedyStepToward(Int2D target) {
        if (target == null) return null;
        TWDirection[] dirs = {TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W};
        TWDirection bestDir = null;
        int bestDist = manhattan(getX(), getY(), target.x, target.y);

        for (TWDirection d : dirs) {
            int nx = getX() + d.dx;
            int ny = getY() + d.dy;
            if (this.getEnvironment().isInBounds(nx, ny) && !isBlocked(nx, ny)) {
                int dist = manhattan(nx, ny, target.x, target.y);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestDir = d;
                }
            }
        }

        if (bestDir != null) {
            return new TWThought(TWAction.MOVE, bestDir);
        }
        return null;
    }

    /**
     * 兜底：随机走一步有效方向（永远不要停）
     */
    private TWThought randomStep() {
        TWDirection[] dirs = {TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W};
        List<TWDirection> valid = new ArrayList<TWDirection>();
        for (TWDirection d : dirs) {
            int nx = getX() + d.dx;
            int ny = getY() + d.dy;
            if (this.getEnvironment().isInBounds(nx, ny) && !isBlocked(nx, ny)) {
                valid.add(d);
            }
        }
        if (valid.isEmpty()) {
            return waitThought();
        }
        TWDirection pick = valid.get(this.getEnvironment().random.nextInt(valid.size()));
        return new TWThought(TWAction.MOVE, pick);
    }

    // ================================================================
    //  通信（通过 Base 的 appendCustomMessages 钩子）
    // ================================================================
    @Override
    protected void appendCustomMessages(List<Message> outbox) {
        long step = this.getEnvironment().schedule.getSteps();
        // 续租目标锁：快过期时重新广播
        if (lockedTargetCell != null && step >= lockedTargetExpiry - 3) {
            outbox.add(createProtocolMessage(
                    CommType.TARGET_LOCK,
                    lockedTargetCell.x,
                    lockedTargetCell.y,
                    encodeTargetLockPayload(0.5)));
            lockedTargetStep = step;
            lockedTargetExpiry = step + TARGET_LOCK_TTL;
        }
    }

    // ================================================================
    //  协议通信：处理队友消息 & 目标锁定
    // ================================================================

    private void processIncomingMessages() {
        long step = this.getEnvironment().schedule.getSteps();

        // 清理过期的队友锁定
        Iterator<Map.Entry<String, Long>> it = teammateLockedTargets.entrySet().iterator();
        while (it.hasNext()) {
            if (it.next().getValue() <= step) {
                it.remove();
            }
        }

        List<Message> inbox = this.getEnvironment().getMessages();
        for (Message message : inbox) {
            ParsedMessage parsed = parseProtocolMessage(message);
            if (parsed == null) continue;
            if (parsed.from == null || parsed.from.equals(this.getName())) continue;

            switch (parsed.type) {
                case OBS_NEW_TILE:
                    // 队友发现新 Tile → 写入记忆
                    if (this.getEnvironment().isInBounds(parsed.x, parsed.y)) {
                        Object existing = this.memory.getMemoryGrid().get(parsed.x, parsed.y);
                        if (existing == null) {
                            // 只记录位置信息，不创建实体（因为没法构造带正确参数的 TWTile）
                        }
                    }
                    break;
                case OBS_NEW_HOLE:
                    break;
                case OBS_FUEL_ONCE:
                    // 队友广播燃料站位置 → 写入记忆
                    if (this.getEnvironment().isInBounds(parsed.x, parsed.y)) {
                        Object fuel = this.getEnvironment().getObjectGrid().get(parsed.x, parsed.y);
                        if (fuel instanceof TWFuelStation
                                && !(this.memory.getMemoryGrid().get(parsed.x, parsed.y) instanceof TWFuelStation)) {
                            this.memory.getMemoryGrid().set(parsed.x, parsed.y, fuel);
                        }
                    }
                    break;
                case TARGET_LOCK:
                    // 队友锁定了某个目标
                    String lockKey = parsed.x + "," + parsed.y;
                    teammateLockedTargets.put(lockKey, step + TARGET_LOCK_TTL + 5);
                    break;
                case TARGET_RELEASE:
                    // 队友释放了某个目标
                    String releaseKey = parsed.x + "," + parsed.y;
                    teammateLockedTargets.remove(releaseKey);
                    break;
                case ACTION_PICKUP_TILE:
                    // 队友捡了 Tile → 从记忆中移除
                    if (this.getEnvironment().isInBounds(parsed.x, parsed.y)) {
                        Object obj = this.memory.getMemoryGrid().get(parsed.x, parsed.y);
                        if (obj instanceof TWTile) {
                            this.memory.getMemoryGrid().set(parsed.x, parsed.y, null);
                            this.memory.removeAgentPercept(parsed.x, parsed.y);
                        }
                    }
                    break;
                case ACTION_FILL_HOLE:
                    // 队友填了 Hole → 从记忆中移除
                    if (this.getEnvironment().isInBounds(parsed.x, parsed.y)) {
                        Object obj = this.memory.getMemoryGrid().get(parsed.x, parsed.y);
                        if (obj instanceof TWHole) {
                            this.memory.getMemoryGrid().set(parsed.x, parsed.y, null);
                            this.memory.removeAgentPercept(parsed.x, parsed.y);
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    }

    private boolean isLockedByTeammate(int x, int y) {
        return teammateLockedTargets.containsKey(x + "," + y);
    }

    private void acquireOwnTargetLock(Int2D pos, double utility) {
        // 释放旧锁
        releaseOwnTargetLock("switch");
        lockedTargetCell = pos;
        long step = this.getEnvironment().schedule.getSteps();
        lockedTargetStep = step;
        lockedTargetExpiry = step + TARGET_LOCK_TTL;
        // 立即广播锁定
        double priority = Math.min(1.0, utility / UTILITY_REFUEL); // 归一化优先级
        publishProtocolMessage(CommType.TARGET_LOCK, pos.x, pos.y,
                encodeTargetLockPayload(priority));
    }

    private void releaseOwnTargetLock(String reason) {
        if (lockedTargetCell == null) return;
        publishProtocolMessage(CommType.TARGET_RELEASE,
                lockedTargetCell.x, lockedTargetCell.y,
                reason != null ? reason : "");
        lockedTargetCell = null;
        lockedTargetStep = -1;
        lockedTargetExpiry = -1;
    }

    // ================================================================
    //  act 覆写
    // ================================================================
    @Override
    protected void act(TWThought thought) {
        stepCount++;
        if (thought != null) {
            String action = thought.getAction().name();
            String dir = (thought.getAction() == TWAction.MOVE)
                    ? thought.getDirection().name() : "-";
            System.out.println(String.format(
                    "[%s #%d] %s %s | pos=(%d,%d) fuel=%d score=%d tiles=%d target=%s",
                    getName(), stepCount, action, dir,
                    getX(), getY(), (int) getFuelLevel(), getScore(),
                    carriedTiles.size(),
                    bestTarget != null ? "(" + bestTarget.x + "," + bestTarget.y + ")" : "none"));
        }
        super.act(thought);
    }
}
