package tileworld.agent;

import java.util.*;

import sim.field.grid.ObjectGrid2D;
import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.*;
import tileworld.exceptions.CellBlockedException;

/**
 * MyAgent - 智能Tileworld智能体实现
 * 
 * 实现优先级决策逻辑：
 * 1. 求生模式（fuel < 150）
 * 2. 填洞模式（有Tile且知道Hole位置）
 * 3. 拾取模式（空间未满且有Tile）
 * 4. 探索模式（犁地式扫描）
 */
public class MyAgent extends TWAgent {
    
    private String name;
    
    // 犁地式扫描相关状态
    private int scanRow = 3; // 从第四行开始（y=3），传感器范围3可以覆盖y=0到y=6（7个单位）
    private int scanCol = 3; // 从第四列开始（x=3），传感器范围3可以覆盖x=0到x=6（7个单位）
    private boolean scanDirectionRight = true; // true = 向右, false = 向左
    private static final int SCAN_STEP = 7; // 犁地式扫描步长
    
    // 重规划相关
    private int replanAttempts = 0; // 重规划尝试次数
    private static final int MAX_REPLAN_ATTEMPTS = 3; // 最大重规划次数
    
    // A*路径规划相关
    private List<TWDirection> currentPath = null;
    private Int2D targetPosition = null;
    
    // 目标对象（用于追踪）
    private TWEntity targetEntity = null;
    
    public MyAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
    }
    
    @Override
    public String getName() {
        return name;
    }
    
    @Override
    public void communicate() {
        // 当前实现不进行通信，但保留接口
        Message message = new Message("", "", "");
        this.getEnvironment().receiveMessage(message);
    }
    
    @Override
    protected TWThought think() {
        // 检查视野内的对象并记录（优先处理FuelStation）
        logVisibleObjects();
        
        // 优先级1: 求生模式 - fuel < 180（停止所有其他任务）
        if (this.getFuelLevel() < 180) {
            System.out.println(String.format("[%s] [模式: 求生] Fuel=%d < 180, 进入求生模式，停止所有其他任务", 
                this.name, (int)this.getFuelLevel()));
            // 清除所有其他任务的路径和目标
            currentPath = null;
            targetPosition = null;
            return thinkSurvivalMode();
        }
        
        // 优先级2: 填洞模式 - 有Tile且记忆中有Hole
        if (this.hasTile() && hasKnownHole()) {
            System.out.println(String.format("[%s] [模式: 填洞] 携带Tile数=%d, 已知Hole存在", 
                this.name, this.carriedTiles.size()));
            return thinkFillHoleMode();
        }
        
        // 优先级3: 拾取模式 - 空间未满且视野内有Tile
        if (this.carriedTiles.size() < 3) {
            TWTile visibleTile = (TWTile) this.memory.getClosestObjectInSensorRange(TWTile.class);
            if (visibleTile != null) {
                System.out.println(String.format("[%s] [模式: 拾取] 空间未满(当前=%d/3), 视野内有Tile在(%d,%d)", 
                    this.name, this.carriedTiles.size(), visibleTile.getX(), visibleTile.getY()));
                return thinkPickupMode(visibleTile);
            }
        }
        
        // 优先级4: 探索模式 - 犁地式扫描
        System.out.println(String.format("[%s] [模式: 探索] 位置=(%d,%d), 扫描目标=(%d,%d)", 
            this.name, this.getX(), this.getY(), scanCol, scanRow));
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
                                this.name, x, y, this.getDistanceTo(fuelStation)));
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
                this.name, tile.getX(), tile.getY(), this.getDistanceTo(tile)));
        }
        if (hole != null) {
            System.out.println(String.format("[%s] [感知] 看到Hole在(%d,%d), 距离=%.1f [已标记到记忆]", 
                this.name, hole.getX(), hole.getY(), this.getDistanceTo(hole)));
        }
    }
    
    /**
     * 求生模式：寻找燃料站（停止所有其他任务）
     */
    private TWThought thinkSurvivalMode() {
        // 检查是否已经在燃料站
        if (this.getEnvironment().inFuelStation(this)) {
            System.out.println(String.format("[%s] [求生模式] 已在燃料站位置，执行加油", this.name));
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }
        
        // 尝试从记忆中查找燃料站
        TWFuelStation fuelStation = findFuelStationInMemory();
        
        if (fuelStation != null) {
            System.out.println(String.format("[%s] [求生模式] 从记忆中找到FuelStation在(%d,%d)，规划A*路径", 
                this.name, fuelStation.getX(), fuelStation.getY()));
            // 已知燃料站位置，使用A*规划路径（带重规划）
            Int2D target = new Int2D(fuelStation.getX(), fuelStation.getY());
            List<TWDirection> path = aStarPathfindingWithReplan(new Int2D(this.getX(), this.getY()), target, fuelStation);
            
            if (path != null && !path.isEmpty()) {
                this.currentPath = path;
                this.targetPosition = target;
                System.out.println(String.format("[%s] [求生模式] A*路径规划成功，路径长度=%d，下一步方向=%s", 
                    this.name, path.size(), path.get(0)));
                return new TWThought(TWAction.MOVE, path.get(0));
            } else {
                System.out.println(String.format("[%s] [求生模式] A*路径规划失败，使用感知找洞模式寻找FuelStation", this.name));
                // A*失败，使用感知找洞模式
                return thinkSurvivalExplorationMode();
            }
        } else {
            System.out.println(String.format("[%s] [求生模式] 记忆中未找到FuelStation，使用感知找洞模式寻找", this.name));
            // 未知燃料站位置，使用感知找洞模式寻找
            return thinkSurvivalExplorationMode();
        }
    }
    
    /**
     * 求生模式的探索：优先寻找FuelStation
     */
    private TWThought thinkSurvivalExplorationMode() {
        // 如果当前有路径且目标位置有效，继续执行
        if (currentPath != null && !currentPath.isEmpty() && targetPosition != null) {
            // 检查是否已经到达目标
            if (this.getX() == targetPosition.x && this.getY() == targetPosition.y) {
                currentPath = null;
                targetPosition = null;
            } else {
                TWDirection nextDir = currentPath.remove(0);
                System.out.println(String.format("[%s] [求生探索] 继续执行路径，剩余步数=%d，下一步=%s", 
                    this.name, currentPath.size(), nextDir));
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
                        this.name, maxY));
                }
            }
            
            System.out.println(String.format("[%s] [求生探索] 计算扫描目标=(%d,%d)，当前位置=(%d,%d)", 
                this.name, nextScanPos.x, nextScanPos.y, this.getX(), this.getY()));
            // 规划路径到扫描位置
            List<TWDirection> path = aStarPathfinding(new Int2D(this.getX(), this.getY()), nextScanPos);
            
            if (path != null && !path.isEmpty()) {
                this.currentPath = new ArrayList<>(path);
                this.targetPosition = nextScanPos;
                System.out.println(String.format("[%s] [求生探索] A*路径规划成功，路径长度=%d，下一步=%s", 
                    this.name, path.size(), path.get(0)));
                return new TWThought(TWAction.MOVE, path.get(0));
            } else {
                System.out.println(String.format("[%s] [求生探索] A*路径规划失败，无法到达扫描目标，使用随机方向", this.name));
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
                System.out.println(String.format("[%s] [求生探索] 限制移动方向，避免超过y=46", this.name));
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
            this.name, thought.getDirection()));
        return thought;
    }
    
    /**
     * 填洞模式：带着Tile去填洞
     */
    private TWThought thinkFillHoleMode() {
        // 查找最近的已知Hole
        TWHole nearestHole = this.memory.getNearbyHole(this.getX(), this.getY(), Double.MAX_VALUE);
        
        if (nearestHole != null) {
            System.out.println(String.format("[%s] [填洞模式] 找到最近Hole在(%d,%d)，当前位置=(%d,%d)，距离=%.1f", 
                this.name, nearestHole.getX(), nearestHole.getY(), 
                this.getX(), this.getY(), this.getDistanceTo(nearestHole)));
            
            // 检查是否已经在Hole位置
            if (this.getX() == nearestHole.getX() && this.getY() == nearestHole.getY()) {
                // 验证当前位置是否真的有Hole（因为对象可能会消失）
                Object objAtPos = this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                if (objAtPos instanceof TWHole) {
                    System.out.println(String.format("[%s] [填洞模式] 已在Hole位置，执行DROP", this.name));
                    return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
                } else {
                    // Hole已经消失，从记忆中移除
                    System.out.println(String.format("[%s] [填洞模式] Hole在(%d,%d)已消失，从记忆中移除", 
                        this.name, nearestHole.getX(), nearestHole.getY()));
                    this.memory.removeObject(nearestHole);
                    // 清除当前路径和目标
                    currentPath = null;
                    targetPosition = null;
                    // 查找下一个Hole
                    return findNextHoleAndPlan();
                }
            }
            
            // 验证目标Hole是否仍然存在（在规划路径前）
            Object objAtTarget = this.getEnvironment().getObjectGrid().get(nearestHole.getX(), nearestHole.getY());
            if (!(objAtTarget instanceof TWHole)) {
                // Hole已经消失，从记忆中移除
                System.out.println(String.format("[%s] [填洞模式] 目标Hole在(%d,%d)已消失，从记忆中移除", 
                    this.name, nearestHole.getX(), nearestHole.getY()));
                this.memory.removeObject(nearestHole);
                // 清除当前路径和目标
                currentPath = null;
                targetPosition = null;
                // 查找下一个Hole
                return findNextHoleAndPlan();
            }
            
            // 在移动过程中，如果视野内有Tile且空间未满，先拾取
            TWTile visibleTile = (TWTile) this.memory.getClosestObjectInSensorRange(TWTile.class);
            if (visibleTile != null && this.carriedTiles.size() < 3) {
                // 如果Tile就在当前位置，先拾取
                if (this.getX() == visibleTile.getX() && this.getY() == visibleTile.getY()) {
                    System.out.println(String.format("[%s] [填洞模式] 当前位置有Tile，先拾取", this.name));
                    return new TWThought(TWAction.PICKUP, TWDirection.Z);
                }
            }
            
            // 检查是否有未完成的路径
            if (currentPath != null && !currentPath.isEmpty() && targetPosition != null) {
                if (targetPosition.x == nearestHole.getX() && targetPosition.y == nearestHole.getY()) {
                    // 继续执行已有路径
                    TWDirection nextDir = currentPath.remove(0);
                    System.out.println(String.format("[%s] [填洞模式] 继续执行已有路径，剩余步数=%d，下一步=%s", 
                        this.name, currentPath.size(), nextDir));
                    return new TWThought(TWAction.MOVE, nextDir);
                } else {
                    // 目标已改变，重新规划
                    System.out.println(String.format("[%s] [填洞模式] 目标已改变，清除旧路径", this.name));
                    currentPath = null;
                    targetPosition = null;
                }
            }
            
            // 规划去Hole的路径（带重规划）
            Int2D target = new Int2D(nearestHole.getX(), nearestHole.getY());
            System.out.println(String.format("[%s] [填洞模式] 规划A*路径到Hole(%d,%d)", 
                this.name, target.x, target.y));
            List<TWDirection> path = aStarPathfindingWithReplan(new Int2D(this.getX(), this.getY()), target, nearestHole);
            
            if (path != null && !path.isEmpty()) {
                this.currentPath = path;
                this.targetPosition = target;
                System.out.println(String.format("[%s] [填洞模式] A*路径规划成功，路径长度=%d，下一步=%s", 
                    this.name, path.size(), path.get(0)));
                return new TWThought(TWAction.MOVE, path.get(0));
            } else {
                System.out.println(String.format("[%s] [填洞模式] A*路径规划失败，无法到达Hole", this.name));
            }
        } else {
            System.out.println(String.format("[%s] [填洞模式] 记忆中未找到Hole，切换到探索模式", this.name));
        }
        
        // 如果找不到Hole，回到探索模式
        return thinkExplorationMode();
    }
    
    /**
     * 查找下一个Hole并规划路径
     */
    private TWThought findNextHoleAndPlan() {
        // 查找下一个最近的Hole
        TWHole nextHole = this.memory.getNearbyHole(this.getX(), this.getY(), Double.MAX_VALUE);
        
        if (nextHole != null) {
            System.out.println(String.format("[%s] [填洞模式] 找到下一个Hole在(%d,%d)，距离=%.1f", 
                this.name, nextHole.getX(), nextHole.getY(), this.getDistanceTo(nextHole)));
            
            // 验证这个Hole是否仍然存在
            Object objAtTarget = this.getEnvironment().getObjectGrid().get(nextHole.getX(), nextHole.getY());
            if (!(objAtTarget instanceof TWHole)) {
                // 这个Hole也消失了，递归查找下一个
                System.out.println(String.format("[%s] [填洞模式] Hole在(%d,%d)也已消失，继续查找", 
                    this.name, nextHole.getX(), nextHole.getY()));
                this.memory.removeObject(nextHole);
                return findNextHoleAndPlan();
            }
            
            // 规划路径（带重规划）
            Int2D target = new Int2D(nextHole.getX(), nextHole.getY());
            List<TWDirection> path = aStarPathfindingWithReplan(new Int2D(this.getX(), this.getY()), target, nextHole);
            
            if (path != null && !path.isEmpty()) {
                this.currentPath = path;
                this.targetPosition = target;
                System.out.println(String.format("[%s] [填洞模式] 规划到下一个Hole的路径成功，路径长度=%d，下一步=%s", 
                    this.name, path.size(), path.get(0)));
                return new TWThought(TWAction.MOVE, path.get(0));
            }
        }
        
        // 如果记忆中没有其他Hole了，切换到探索模式
        System.out.println(String.format("[%s] [填洞模式] 记忆中没有其他Hole，切换到探索模式寻找新Hole", this.name));
        return thinkExplorationMode();
    }
    
    /**
     * 拾取模式：拾取Tile
     */
    private TWThought thinkPickupMode(TWTile tile) {
        // 检查是否已经在Tile位置
        if (this.getX() == tile.getX() && this.getY() == tile.getY()) {
            System.out.println(String.format("[%s] [拾取模式] 已在Tile位置，执行PICKUP", this.name));
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }
        
        // 检查是否有未完成的路径
        if (currentPath != null && !currentPath.isEmpty() && targetPosition != null) {
            if (targetPosition.x == tile.getX() && targetPosition.y == tile.getY()) {
                // 继续执行已有路径
                TWDirection nextDir = currentPath.remove(0);
                System.out.println(String.format("[%s] [拾取模式] 继续执行已有路径，剩余步数=%d，下一步=%s", 
                    this.name, currentPath.size(), nextDir));
                return new TWThought(TWAction.MOVE, nextDir);
            }
        }
        
        // 规划去Tile的路径（带重规划）
        Int2D target = new Int2D(tile.getX(), tile.getY());
        System.out.println(String.format("[%s] [拾取模式] 规划A*路径到Tile(%d,%d)", 
            this.name, target.x, target.y));
        List<TWDirection> path = aStarPathfindingWithReplan(new Int2D(this.getX(), this.getY()), target, tile);
        
        if (path != null && !path.isEmpty()) {
            this.currentPath = path;
            this.targetPosition = target;
            System.out.println(String.format("[%s] [拾取模式] A*路径规划成功，路径长度=%d，下一步=%s", 
                this.name, path.size(), path.get(0)));
            return new TWThought(TWAction.MOVE, path.get(0));
        } else {
            System.out.println(String.format("[%s] [拾取模式] A*路径规划失败，无法到达Tile，切换到探索模式", this.name));
        }
        
        // 如果无法到达，回到探索模式
        return thinkExplorationMode();
    }
    
    /**
     * 探索模式：犁地式扫描
     */
    private TWThought thinkExplorationMode() {
        // 如果当前有路径且目标位置有效，继续执行
        if (currentPath != null && !currentPath.isEmpty() && targetPosition != null) {
            // 检查是否已经到达目标
            if (this.getX() == targetPosition.x && this.getY() == targetPosition.y) {
                System.out.println(String.format("[%s] [探索模式] 已到达扫描目标(%d,%d)，清除路径", 
                    this.name, targetPosition.x, targetPosition.y));
                currentPath = null;
                targetPosition = null;
            } else {
                TWDirection nextDir = currentPath.remove(0);
                System.out.println(String.format("[%s] [探索模式] 继续执行路径，剩余步数=%d，下一步=%s，目标=(%d,%d)", 
                    this.name, currentPath.size(), nextDir, targetPosition.x, targetPosition.y));
                return new TWThought(TWAction.MOVE, nextDir);
            }
        }
        
        // 计算下一个扫描目标位置
        Int2D nextScanPos = calculateNextScanPosition();
        
        if (nextScanPos != null) {
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
                        this.name, maxY));
                }
            }
            
            System.out.println(String.format("[%s] [探索模式] 计算扫描目标=(%d,%d)，当前位置=(%d,%d)", 
                this.name, nextScanPos.x, nextScanPos.y, this.getX(), this.getY()));
            // 规划路径到扫描位置
            List<TWDirection> path = aStarPathfinding(new Int2D(this.getX(), this.getY()), nextScanPos);
            
            if (path != null && !path.isEmpty()) {
                this.currentPath = new ArrayList<>(path); // 创建副本，避免直接修改
                this.targetPosition = nextScanPos;
                System.out.println(String.format("[%s] [探索模式] A*路径规划成功，路径长度=%d，下一步=%s", 
                    this.name, path.size(), path.get(0)));
                return new TWThought(TWAction.MOVE, path.get(0));
            } else {
                System.out.println(String.format("[%s] [探索模式] A*路径规划失败，无法到达扫描目标，使用随机方向", this.name));
            }
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
                System.out.println(String.format("[%s] [探索模式] 限制移动方向，避免超过y=46", this.name));
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
            this.name, thought.getDirection()));
        return thought;
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
                                this.name, x, y));
                        }
                    } else if (memObj instanceof TWObstacle && obj == null) {
                        // 如果记忆中标记为障碍物，但实际环境中已不存在，清除记忆
                        memoryGrid.set(x, y, null);
                        System.out.println(String.format("[%s] [重规划] 障碍物在(%d,%d)已消失，清除记忆", 
                            this.name, x, y));
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
                this.name, pathCost, this.getFuelLevel()));
            return false;
        }
        
        // 检查2: 如果目标有生命周期，检查是否能在生命周期内到达
        if (targetEntity instanceof TWObject) {
            TWObject obj = (TWObject) targetEntity;
            double currentTime = this.getEnvironment().schedule.getTime();
            double timeLeft = obj.getTimeLeft(currentTime);
            
            if (timeLeft < pathCost) {
                System.out.println(String.format("[%s] [重规划] 路径成本(%d)超过目标剩余生命周期(%.1f)，放弃目标", 
                    this.name, pathCost, timeLeft));
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
            System.out.println(String.format("[%s] [重规划] 第%d次重规划尝试", this.name, attempt));
            
            // 刷新记忆中的障碍物
            refreshObstacleMemory();
            
            // 重新运行A*
            path = aStarPathfinding(start, goal, targetEntity);
            
            if (path != null && evaluatePathCost(path, targetEntity)) {
                System.out.println(String.format("[%s] [重规划] 重规划成功，路径长度=%d", this.name, path.size()));
                replanAttempts = 0; // 重置重规划次数
                return path;
            }
        }
        
        System.out.println(String.format("[%s] [重规划] 重规划失败，已达到最大尝试次数(%d)，放弃目标", 
            this.name, MAX_REPLAN_ATTEMPTS));
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
     * 从记忆中查找燃料站
     */
    private TWFuelStation findFuelStationInMemory() {
        sim.field.grid.ObjectGrid2D memoryGrid = this.memory.getMemoryGrid();
        int xDim = this.getEnvironment().getxDimension();
        int yDim = this.getEnvironment().getyDimension();
        
        // 遍历记忆网格查找燃料站
        for (int x = 0; x < xDim; x++) {
            for (int y = 0; y < yDim; y++) {
                Object obj = memoryGrid.get(x, y);
                if (obj instanceof TWFuelStation) {
                    return (TWFuelStation) obj;
                }
            }
        }
        
        return null;
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
        
        TWDirection chosenDir = validDirs.get(this.getEnvironment().random.nextInt(validDirs.size()));
        return new TWThought(TWAction.MOVE, chosenDir);
    }
    
    @Override
    protected void act(TWThought thought) {
        try {
            switch (thought.getAction()) {
                case MOVE:
                    int oldX = this.getX();
                    int oldY = this.getY();
                    this.move(thought.getDirection());
                    System.out.println(String.format("[%s] [执行] MOVE %s: (%d,%d) -> (%d,%d), Fuel=%d", 
                        this.name, thought.getDirection(), oldX, oldY, this.getX(), this.getY(), (int)this.getFuelLevel()));
                    // 如果到达目标位置，清除路径
                    if (targetPosition != null && 
                        this.getX() == targetPosition.x && 
                        this.getY() == targetPosition.y) {
                        System.out.println(String.format("[%s] [执行] 已到达目标位置(%d,%d)，清除路径", 
                            this.name, targetPosition.x, targetPosition.y));
                        currentPath = null;
                        targetPosition = null;
                    }
                    break;
                    
                case PICKUP:
                    // 从环境中直接获取当前位置的Tile
                    TWTile tile = (TWTile) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                    if (tile instanceof TWTile && this.carriedTiles.size() < 3) {
                        System.out.println(String.format("[%s] [执行] PICKUP Tile在(%d,%d)，当前携带数=%d", 
                            this.name, this.getX(), this.getY(), this.carriedTiles.size()));
                        this.pickUpTile(tile);
                        System.out.println(String.format("[%s] [执行] PICKUP完成，当前携带数=%d", 
                            this.name, this.carriedTiles.size()));
                    } else {
                        System.out.println(String.format("[%s] [执行] PICKUP失败：Tile不存在或空间已满(当前=%d/3)", 
                            this.name, this.carriedTiles.size()));
                    }
                    break;
                    
                case PUTDOWN:
                    // 从环境中直接获取当前位置的Hole
                    TWHole hole = (TWHole) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                    if (hole instanceof TWHole && this.hasTile()) {
                        int oldScore = this.score;
                        System.out.println(String.format("[%s] [执行] PUTDOWN Tile到Hole在(%d,%d)，当前携带数=%d，当前分数=%d", 
                            this.name, this.getX(), this.getY(), this.carriedTiles.size(), oldScore));
                        this.putTileInHole(hole);
                        System.out.println(String.format("[%s] [执行] PUTDOWN完成，当前携带数=%d，新分数=%d", 
                            this.name, this.carriedTiles.size(), this.score));
                        // 填洞成功后，清除路径和目标
                        currentPath = null;
                        targetPosition = null;
                    } else {
                        System.out.println(String.format("[%s] [执行] PUTDOWN失败：Hole不存在或没有Tile(携带数=%d)", 
                            this.name, this.carriedTiles.size()));
                        // 如果Hole不存在，从记忆中移除当前位置的Hole（如果存在）
                        TWHole memHole = this.memory.getNearbyHole(this.getX(), this.getY(), Double.MAX_VALUE);
                        if (memHole != null && memHole.getX() == this.getX() && memHole.getY() == this.getY()) {
                            System.out.println(String.format("[%s] [执行] 从记忆中移除已消失的Hole在(%d,%d)", 
                                this.name, this.getX(), this.getY()));
                            this.memory.removeObject(memHole);
                        }
                        // 清除路径和目标
                        currentPath = null;
                        targetPosition = null;
                    }
                    break;
                    
                case REFUEL:
                    if (this.getEnvironment().inFuelStation(this)) {
                        double oldFuel = this.getFuelLevel();
                        System.out.println(String.format("[%s] [执行] REFUEL，当前Fuel=%.1f", this.name, oldFuel));
                        this.refuel();
                        System.out.println(String.format("[%s] [执行] REFUEL完成，新Fuel=%.1f", 
                            this.name, this.getFuelLevel()));
                    } else {
                        System.out.println(String.format("[%s] [执行] REFUEL失败：不在燃料站位置，当前位置=(%d,%d)", 
                            this.name, this.getX(), this.getY()));
                    }
                    break;
            }
        } catch (CellBlockedException ex) {
            // 单元格被阻挡，清除当前路径，重新规划
            System.out.println(String.format("[%s] [错误] CellBlockedException: 单元格被阻挡，清除路径，位置=(%d,%d)", 
                this.name, this.getX(), this.getY()));
            currentPath = null;
            targetPosition = null;
        }
    }
}
