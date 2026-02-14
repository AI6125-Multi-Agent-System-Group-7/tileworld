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
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;

/**
 * Group7AgentBase
 *
 * Let's put shared utilities for all Group 7 agents in this class!
 * Keep your policy small; keep your robots alive.
 */
public abstract class Group7AgentBase extends TWAgent {
    private final String name;

    protected List<TWDirection> currentPath;
    protected Int2D currentTarget;

    public Group7AgentBase(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
        this.currentPath = null;
        this.currentTarget = null;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void communicate() {
        //TODO: Override in our own agent class later
        Message message = new Message("", "", "");
        this.getEnvironment().receiveMessage(message);
    }

    /**
     * Call this near the start of think() to store fuel station in memory.
     * The sensor ignores it because it is not a TWObject. Sneaky, right?
     */
    protected void rememberFuelStationsInSensorRange() {
        int r = Parameters.defaultSensorRange;
        ObjectGrid2D objectGrid = this.getEnvironment().getObjectGrid();
        ObjectGrid2D memoryGrid = this.memory.getMemoryGrid();

        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int x = this.getX() + dx;
                int y = this.getY() + dy;
                if (!this.getEnvironment().isInBounds(x, y)) {
                    continue;
                }

                Object obj = objectGrid.get(x, y);
                if (obj instanceof TWFuelStation) {
                    if (!(memoryGrid.get(x, y) instanceof TWFuelStation)) {
                        memoryGrid.set(x, y, obj);
                    }
                }
            }
        }
    }

    /**
     * Finds a remembered fuel station (if any). The station does not move,
     * unlike your motivation during report season.
     */
    protected TWFuelStation findFuelStationInMemory() {
        ObjectGrid2D memoryGrid = this.memory.getMemoryGrid();
        int xDim = this.getEnvironment().getxDimension();
        int yDim = this.getEnvironment().getyDimension();

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
     * Fuel check based on distance to the station. 
     * If you cannot afford the trip, do not buy the ticket.
     */
    protected boolean shouldRefuel(int safetyMargin) {
        TWFuelStation station = findFuelStationInMemory();
        if (station == null) {
            return this.getFuelLevel() <= safetyMargin;
        }
        int dist = (int) this.getDistanceTo(station);
        return this.getFuelLevel() <= dist + safetyMargin;
    }

    // ================ Utilities ================
    protected TWTile nearestKnownTile() {
        return this.memory.getNearbyTile(this.getX(), this.getY(), Double.MAX_VALUE);
    }

    protected TWHole nearestKnownHole() {
        return this.memory.getNearbyHole(this.getX(), this.getY(), Double.MAX_VALUE);
    }

    protected TWTile closestVisibleTile() {
        return (TWTile) this.memory.getClosestObjectInSensorRange(TWTile.class);
    }

    protected TWHole closestVisibleHole() {
        return (TWHole) this.memory.getClosestObjectInSensorRange(TWHole.class);
    }

    protected TWThought waitThought() {
        return new TWThought(TWAction.MOVE, TWDirection.Z);
    }

    protected void clearPlan() {
        currentPath = null;
        currentTarget = null;
    }

    /**
     * Plan a path if needed and step along it. Simple, fast, and easy to debug.
     */
    protected TWThought stepToward(Int2D target) {
        if (target == null) {
            return waitThought();
        }

        if (currentTarget == null || !currentTarget.equals(target) || currentPath == null || currentPath.isEmpty()) {
            currentTarget = target;
            currentPath = planPath(new Int2D(this.getX(), this.getY()), target);
        }

        if (currentPath == null || currentPath.isEmpty()) {
            return waitThought();
        }

        TWDirection next = currentPath.remove(0);
        return new TWThought(TWAction.MOVE, next);
    }

    /**
     * A* pathfinding with Manhattan distance. It is not magic. It just looks that way.
     */
    protected List<TWDirection> planPath(Int2D start, Int2D goal) {
        if (start.equals(goal)) {
            return new ArrayList<TWDirection>();
        }

        PriorityQueue<AStarNode> open = new PriorityQueue<>(Comparator.comparingInt(n -> n.fCost));
        Set<Int2D> closed = new HashSet<>();
        Map<Int2D, AStarNode> nodes = new HashMap<>();

        AStarNode startNode = new AStarNode(start, null, 0, manhattan(start, goal));
        open.add(startNode);
        nodes.put(start, startNode);

        while (!open.isEmpty()) {
            AStarNode current = open.poll();
            if (current.pos.equals(goal)) {
                return reconstructPath(current);
            }

            closed.add(current.pos);

            for (TWDirection dir : CARDINALS) {
                Int2D next = new Int2D(current.pos.x + dir.dx, current.pos.y + dir.dy);
                if (!this.getEnvironment().isInBounds(next.x, next.y)) {
                    continue;
                }
                if (closed.contains(next)) {
                    continue;
                }
                if (isBlocked(next.x, next.y)) {
                    continue;
                }

                int g = current.gCost + 1;
                AStarNode nextNode = nodes.get(next);
                if (nextNode == null) {
                    nextNode = new AStarNode(next, current, g, manhattan(next, goal));
                    nodes.put(next, nextNode);
                    open.add(nextNode);
                } else if (g < nextNode.gCost) {
                    nextNode.gCost = g;
                    nextNode.fCost = g + nextNode.hCost;
                    nextNode.parent = current;
                    open.remove(nextNode);
                    open.add(nextNode);
                }
            }
        }

        return null;
    }

    protected boolean isBlocked(int x, int y) {
        if (!this.getEnvironment().isInBounds(x, y)) {
            return true;
        }
        if (this.getEnvironment().isCellBlocked(x, y)) {
            return true;
        }
        return this.memory.isCellBlocked(x, y);
    }

    private List<TWDirection> reconstructPath(AStarNode node) {
        List<TWDirection> path = new ArrayList<>();
        AStarNode current = node;

        while (current.parent != null) {
            Int2D diff = new Int2D(current.pos.x - current.parent.pos.x, current.pos.y - current.parent.pos.y);
            TWDirection dir = directionFromDiff(diff);
            if (dir != null) {
                path.add(0, dir);
            }
            current = current.parent;
        }

        return path;
    }

    private int manhattan(Int2D a, Int2D b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    private TWDirection directionFromDiff(Int2D diff) {
        if (diff.x == 1 && diff.y == 0) return TWDirection.E;
        if (diff.x == -1 && diff.y == 0) return TWDirection.W;
        if (diff.x == 0 && diff.y == -1) return TWDirection.N;
        if (diff.x == 0 && diff.y == 1) return TWDirection.S;
        return null;
    }

    @Override
    protected void act(TWThought thought) {
        if (thought == null) {
            return; // Zen mode: do nothing and pretend it was intentional.
        }

        try {
            switch (thought.getAction()) {
                case MOVE:
                    this.move(thought.getDirection());
                    if (currentTarget != null && this.getX() == currentTarget.x && this.getY() == currentTarget.y) {
                        clearPlan();
                    }
                    break;
                case PICKUP:
                    TWEntity obj = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                    if (obj instanceof TWTile && this.carriedTiles.size() < 3) {
                        this.pickUpTile((TWTile) obj);
                    }
                    break;
                case PUTDOWN:
                    TWEntity hole = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                    if (hole instanceof TWHole && this.hasTile()) {
                        this.putTileInHole((TWHole) hole);
                        clearPlan();
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
        } catch (CellBlockedException ex) {
            clearPlan();
        }
    }

    private static final TWDirection[] CARDINALS = new TWDirection[] {
        TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W
    };

    private static class AStarNode {
        final Int2D pos;
        AStarNode parent;
        int gCost;
        final int hCost;
        int fCost;

        AStarNode(Int2D pos, AStarNode parent, int gCost, int hCost) {
            this.pos = pos;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
            this.fCost = gCost + hCost;
        }
    }
}
