package tileworld.agent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;

/**
 * AgentSun —— a regional explorer Agent
 *
 * Only the think()} method and private helpers are implemented here;
 * communication, pathfinding, action execution and fuel-station memory are
 * all handled by Group7AgentBase.
 */
public class AgentSun extends Group7AgentBase {

    private int stepCount;

    // Fuel thresholds
    private static final int CRITICAL_FUEL_MARGIN         = 20;   // 30
    private static final int OPPORTUNISTIC_FUEL_THRESHOLD = 150;  // 250
    private static final int NEAR_STATION_DISTANCE        = 8;    // 15

    // Memory freshness
    private static final double MEMORY_EXPIRY = Parameters.lifeTime;

    // Lifetime-aware target filtering
    private final Map<String, Double> cellObservedAt = new HashMap<>();
    private static final int FRESHNESS_BUFFER = 5;

    // Regional exploration state
    private final List<Int2D> regionalSearchPoints;
    private int currentRegionIndex;
    private Int2D currentRegionTarget; // stable jittered target per region

    // Cached fuel-station position
    private Int2D fuelStationPosition;

    // Team coordination (refreshed every step)
    private final Set<String> claimedTargets;

    // Anti-stuck
    private int stuckCounter;
    private final Random rng;
    private static final TWDirection[] DIRS = {
        TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W
    };

    public AgentSun(String name, int xpos, int ypos,
                                  TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
        this.regionalSearchPoints = new ArrayList<Int2D>();
        this.currentRegionIndex = 0;
        this.currentRegionTarget = null;
        this.fuelStationPosition = null;
        this.claimedTargets = new HashSet<String>();
        this.stuckCounter = 0;
        this.rng = new Random(name.hashCode());

        initializeRegionalSearchPoints();
    }

   
    // Regional search-point generation
    private void initializeRegionalSearchPoints() {
        int xDim = getEnvironment().getxDimension();
        int yDim = getEnvironment().getyDimension();
        int margin = Parameters.defaultSensorRange + 1;          // 4
        int step   = Parameters.defaultSensorRange * 2 + 1;      // 7

        int left   = margin;
        int right  = xDim - 1 - margin;
        int top    = margin;
        int bottom = yDim - 1 - margin;

        // Boustrophedon (lawn-mower) sweep: alternating left-right strips
        // spaced one sensor-diameter apart, guaranteeing ~85% map coverage.
        boolean goingRight = true;
        int prevY = top - step; // initialise below top so first row is always added
        for (int y = top; y <= bottom; y += step) {
            if (goingRight) {
                regionalSearchPoints.add(new Int2D(left,  y));
                regionalSearchPoints.add(new Int2D(right, y));
            } else {
                regionalSearchPoints.add(new Int2D(right, y));
                regionalSearchPoints.add(new Int2D(left,  y));
            }
            goingRight = !goingRight;
            prevY = y;
        }

        // Ensure the bottom strip is covered when step size doesn't divide evenly.
        if (prevY < bottom - step / 2) {
            if (goingRight) {
                regionalSearchPoints.add(new Int2D(left,  bottom));
                regionalSearchPoints.add(new Int2D(right, bottom));
            } else {
                regionalSearchPoints.add(new Int2D(right, bottom));
                regionalSearchPoints.add(new Int2D(left,  bottom));
            }
        }

        // Safety fallback for very small maps
        if (regionalSearchPoints.isEmpty()) {
            regionalSearchPoints.add(new Int2D(xDim / 2, yDim / 2));
        }
    }


    // act() - logging wrapper
    @Override
    protected void act(TWThought thought) {
        stepCount++;
        String action = (thought == null) ? "NULL" : thought.getAction().name();
        String dir = (thought != null && thought.getAction() == TWAction.MOVE)
                ? thought.getDirection().name()
                : "-";

        System.out.println(String.format(
                "[Sun step %d] %s %s | pos=(%d,%d) fuel=%d score=%d carried=%d",
                stepCount, action, dir,
                this.getX(), this.getY(),
                (int) this.getFuelLevel(),
                this.getScore(),
                this.carriedTiles.size()));
        super.act(thought);
    }


    // think()
    @Override
    protected TWThought think() {
        // Step 0: update fuel-station knowledge and verify memory against sensor reality
        updateFuelStationKnowledge();
        updateObservationTimes();

        // Step 1: parse teammate messages
        parseTeammateMessages();

        // Priority 1: Fuel management
        TWThought fuelDecision = handleFuelManagement();
        if (fuelDecision != null) {
            return ensureNotStuck(fuelDecision);
        }

        // Immediate action on current cell
        TWThought immediateAction = handleImmediateAction();
        if (immediateAction != null) {
            stuckCounter = 0;
            return immediateAction;
        }

        // Priority 2: Task completion (deliver tiles / collect tiles)
        TWThought taskDecision = selectBestTask();
        if (taskDecision != null) {
            return ensureNotStuck(taskDecision);
        }

        // Priority 3: Regional exploration
        TWThought exploreDecision = exploreRegionally();
        return ensureNotStuck(exploreDecision);
    }

    /**
     * If the given thought is a wait (MOVE Z), try greedy/random alternatives.
     * This is the single anti-stuck gate that all movement decisions go through.
     */
    private TWThought ensureNotStuck(TWThought thought) {
        if (!isWaitThought(thought)) {
            stuckCounter = 0;
            return thought;
        }

        stuckCounter++;
        clearPlan();

        // Try greedy step toward any current target
        if (currentTarget != null) {
            TWThought greedy = greedyStepToward(currentTarget);
            if (greedy != null) {
                stuckCounter = 0;
                return greedy;
            }
        }

        // Try random move
        TWThought randomMove = tryRandomMove();
        if (randomMove != null) {
            return randomMove;
        }

        return thought;
    }


    // Movement helpers
    /**
     * Greedy single-step toward target: try the best direction first
     * (reduces manhattan distance), then try others. Bypasses A* entirely.
     * Only checks real environment blocks, not stale memory.
     */
    private TWThought greedyStepToward(Int2D target) {
        int dx = target.x - getX();
        int dy = target.y - getY();

        // Build preference order: best direction first
        List<TWDirection> preferred = new ArrayList<TWDirection>(4);
        if (Math.abs(dx) >= Math.abs(dy)) {
            if (dx > 0) preferred.add(TWDirection.E);
            if (dx < 0) preferred.add(TWDirection.W);
            if (dy > 0) preferred.add(TWDirection.S);
            if (dy < 0) preferred.add(TWDirection.N);
        } else {
            if (dy > 0) preferred.add(TWDirection.S);
            if (dy < 0) preferred.add(TWDirection.N);
            if (dx > 0) preferred.add(TWDirection.E);
            if (dx < 0) preferred.add(TWDirection.W);
        }
        // Add remaining directions
        for (TWDirection d : DIRS) {
            if (!preferred.contains(d)) {
                preferred.add(d);
            }
        }

        for (TWDirection d : preferred) {
            int nx = getX() + d.dx;
            int ny = getY() + d.dy;
            if (getEnvironment().isInBounds(nx, ny)
                    && !getEnvironment().isCellBlocked(nx, ny)) {
                return new TWThought(TWAction.MOVE, d);
            }
        }
        return null;
    }

    private TWThought tryRandomMove() {
        int start = rng.nextInt(4);
        for (int i = 0; i < 4; i++) {
            TWDirection d = DIRS[(start + i) % 4];
            int nx = getX() + d.dx;
            int ny = getY() + d.dy;
            if (getEnvironment().isInBounds(nx, ny)
                    && !getEnvironment().isCellBlocked(nx, ny)) {
                return new TWThought(TWAction.MOVE, d);
            }
        }
        return null;
    }


    // Fuel management
    private TWThought handleFuelManagement() {
        if (getEnvironment().inFuelStation(this) && shouldRefuel(CRITICAL_FUEL_MARGIN)) {
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }
        if (isCriticalFuelLevel()) {
            return navigateToFuelStation();
        }
        if (shouldRefuelOpportunistically()) {
            return navigateToFuelStation();
        }
        return null;
    }

    private boolean isCriticalFuelLevel() {
        if (fuelStationPosition == null) {
            return getFuelLevel() <= CRITICAL_FUEL_MARGIN;
        }
        int dist = manhattan(new Int2D(getX(), getY()), fuelStationPosition);
        return getFuelLevel() <= (dist + CRITICAL_FUEL_MARGIN);
    }

    private boolean shouldRefuelOpportunistically() {
        if (fuelStationPosition == null) {
            return false;
        }
        int dist = manhattan(new Int2D(getX(), getY()), fuelStationPosition);
        if (dist >= NEAR_STATION_DISTANCE || getFuelLevel() >= OPPORTUNISTIC_FUEL_THRESHOLD) {
            return false;
        }
        // Don't interrupt a high-value delivery: carrying tile + hole visible right now
        if (hasTile() && closestVisibleHole() != null) {
            return false;
        }
        return true;
    }

    private TWThought navigateToFuelStation() {
        if (fuelStationPosition != null) {
            TWThought step = stepToward(fuelStationPosition);
            if (isWaitThought(step)) {
                // A* failed - try greedy
                return greedyStepToward(fuelStationPosition);
            }
            return step;
        }
        return exploreZigZag();
    }

    private void updateFuelStationKnowledge() {
        rememberFuelStationsInSensorRange();
        if (fuelStationPosition == null) {
            TWFuelStation station = findFuelStationInMemory();
            if (station != null) {
                fuelStationPosition = new Int2D(station.getX(), station.getY());
            }
        }
    }

    /**
     * Scans the sensor range each step and:
     * (1) Records the first simulation step at which we personally observe each
     *     tile or hole (used by isReachableBeforeExpiry).
     * (2) Performs sensor-based memory verification: within sensor range we have
     *     ground truth, so any memory percept that no longer matches reality is
     *     cleared immediately. This replaces time-based cleanExpiredPercepts and
     *     is more accurate because it uses actual observation rather than age.
     */
    private void updateObservationTimes() {
        double now = getEnvironment().schedule.getTime();
        int r = Parameters.defaultSensorRange;
        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int x = getX() + dx;
                int y = getY() + dy;
                if (!getEnvironment().isInBounds(x, y)) continue;

                Object realObj = getEnvironment().getObjectGrid().get(x, y);
                Object memObj  = memory.getMemoryGrid().get(x, y);

                // Ground-truth verification: clear stale percepts within sensor range.
                // TWFuelStation is excluded - it never expires.
                if (memObj != null && !(memObj instanceof TWFuelStation)) {
                    boolean stale = false;
                    if (memObj instanceof TWObstacle && !(realObj instanceof TWObstacle)) {
                        stale = true;
                    } else if (memObj instanceof TWTile && !(realObj instanceof TWTile)) {
                        stale = true;
                        cellObservedAt.remove(cellKey(x, y));
                    } else if (memObj instanceof TWHole && !(realObj instanceof TWHole)) {
                        stale = true;
                        cellObservedAt.remove(cellKey(x, y));
                    }
                    if (stale) {
                        memory.removeAgentPercept(x, y);
                        memory.getMemoryGrid().set(x, y, null);
                    }
                }

                // Record first observation of current tiles/holes.
                if (realObj instanceof TWTile || realObj instanceof TWHole) {
                    String key = cellKey(x, y);
                    if (!cellObservedAt.containsKey(key)) {
                        cellObservedAt.put(key, now);
                    }
                }
            }
        }
        // Prune cellObservedAt entries that exceed the object lifetime.
        final double threshold = now;
        cellObservedAt.entrySet().removeIf(e -> threshold - e.getValue() >= MEMORY_EXPIRY);
    }


    // Immediate cell actions
    private TWThought handleImmediateAction() {
        TWEntity here = (TWEntity) getEnvironment().getObjectGrid()
                .get(getX(), getY());

        if (here instanceof TWHole && hasTile()) {
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (here instanceof TWTile && carriedTiles.size() < 3) {
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }
        return null;
    }


    // Task selection - with freshness filtering
    private TWThought selectBestTask() {
        if (hasTile()) {
            TWThought holeStep = findAndGoToHole();
            if (holeStep != null) return holeStep;
        }

        if (carriedTiles.size() < 3) {
            TWThought tileStep = findAndGoToTile();
            if (tileStep != null) return tileStep;
        }

        return null;
    }

    private TWThought findAndGoToHole() {
        // Check visible first
        TWHole visibleHole = closestVisibleHole();
        if (visibleHole != null && !isClaimed(visibleHole.getX(), visibleHole.getY())) {
            Int2D target = new Int2D(visibleHole.getX(), visibleHole.getY());
            TWThought step = stepToward(target);
            if (!isWaitThought(step)) return step;
            // A* failed, try greedy
            TWThought greedy = greedyStepToward(target);
            if (greedy != null) return greedy;
        }

        // Check memory - also filter by reachability before expiry
        TWHole knownHole = memory.getNearbyHole(getX(), getY(), MEMORY_EXPIRY);
        if (knownHole != null && !isClaimed(knownHole.getX(), knownHole.getY())
                && isReachableBeforeExpiry(knownHole.getX(), knownHole.getY())) {
            Int2D target = new Int2D(knownHole.getX(), knownHole.getY());
            TWThought step = stepToward(target);
            if (!isWaitThought(step)) return step;
            TWThought greedy = greedyStepToward(target);
            if (greedy != null) return greedy;
        }

        return null;
    }

    private TWThought findAndGoToTile() {
        TWTile visibleTile = closestVisibleTile();
        if (visibleTile != null && !isClaimed(visibleTile.getX(), visibleTile.getY())) {
            if (isFuelSafeForDetour(visibleTile.getX(), visibleTile.getY())) {
                Int2D target = new Int2D(visibleTile.getX(), visibleTile.getY());
                TWThought step = stepToward(target);
                if (!isWaitThought(step)) return step;
                TWThought greedy = greedyStepToward(target);
                if (greedy != null) return greedy;
            }
        }

        // Check memory - also filter by reachability before expiry
        TWTile knownTile = memory.getNearbyTile(getX(), getY(), MEMORY_EXPIRY);
        if (knownTile != null && !isClaimed(knownTile.getX(), knownTile.getY())
                && isFuelSafeForDetour(knownTile.getX(), knownTile.getY())
                && isReachableBeforeExpiry(knownTile.getX(), knownTile.getY())) {
            Int2D target = new Int2D(knownTile.getX(), knownTile.getY());
            TWThought step = stepToward(target);
            if (!isWaitThought(step)) return step;
            TWThought greedy = greedyStepToward(target);
            if (greedy != null) return greedy;
        }

        return null;
    }

    /**
     * Returns true if the target tile/hole is likely to still exist when the
     * agent arrives, based on when we first personally observed it.
     * Conservative: returns true if we have no local observation data (e.g.,
     * the target was learned from a teammate message rather than direct sight).
     */
    private boolean isReachableBeforeExpiry(int targetX, int targetY) {
        String key = cellKey(targetX, targetY);
        Double observedAt = cellObservedAt.get(key);
        if (observedAt == null) {
            return true;
        }
        double age = getEnvironment().schedule.getTime() - observedAt;
        int dist = manhattan(new Int2D(getX(), getY()), new Int2D(targetX, targetY));
        return (age + dist + FRESHNESS_BUFFER) < MEMORY_EXPIRY;
    }

    private boolean isFuelSafeForDetour(int targetX, int targetY) {
        if (fuelStationPosition == null) {
            return manhattan(new Int2D(getX(), getY()),
                             new Int2D(targetX, targetY)) < CRITICAL_FUEL_MARGIN;
        }
        int distToTarget = manhattan(new Int2D(getX(), getY()),
                                     new Int2D(targetX, targetY));
        int distTargetToStation = manhattan(new Int2D(targetX, targetY),
                                            fuelStationPosition);
        return getFuelLevel() > (distToTarget + distTargetToStation + CRITICAL_FUEL_MARGIN);
    }


    // Regional exploration - stable target per region
    private TWThought exploreRegionally() {
        if (regionalSearchPoints.isEmpty()) {
            return exploreZigZag();
        }

        // Compute stable target for current region (only jitter once per region)
        if (currentRegionTarget == null) {
            currentRegionTarget = addJitter(regionalSearchPoints.get(currentRegionIndex));
        }

        // If reached current target, advance to next region
        if (manhattan(new Int2D(getX(), getY()), currentRegionTarget)
                <= Parameters.defaultSensorRange) {
            advanceRegion();
        }

        TWThought step = stepToward(currentRegionTarget);
        if (!isWaitThought(step)) {
            return step;
        }

        // A* failed - try greedy toward the target
        TWThought greedy = greedyStepToward(currentRegionTarget);
        if (greedy != null) {
            return greedy;
        }

        // Both failed - skip this region entirely
        clearPlan();
        advanceRegion();
        return exploreZigZag();
    }

    private void advanceRegion() {
        currentRegionIndex = (currentRegionIndex + 1) % regionalSearchPoints.size();
        currentRegionTarget = addJitter(regionalSearchPoints.get(currentRegionIndex));
    }

    private Int2D addJitter(Int2D target) {
        int range = Parameters.defaultSensorRange;
        int jx = target.x + rng.nextInt(2 * range + 1) - range;
        int jy = target.y + rng.nextInt(2 * range + 1) - range;
        jx = Math.max(0, Math.min(getEnvironment().getxDimension() - 1, jx));
        jy = Math.max(0, Math.min(getEnvironment().getyDimension() - 1, jy));
        return new Int2D(jx, jy);
    }

    private boolean isWaitThought(TWThought thought) {
        return thought != null
                && thought.getAction() == TWAction.MOVE
                && thought.getDirection() == TWDirection.Z;
    }


    // Teammate message parsing
    private void parseTeammateMessages() {
        claimedTargets.clear();

        for (Message msg : getEnvironment().getMessages()) {
            ParsedMessage parsed = parseProtocolMessage(msg);
            if (parsed == null) continue;
            if (parsed.from != null && parsed.from.equals(getName())) continue;

            switch (parsed.type) {
                case ACTION_PICKUP_TILE:
                    claimedTargets.add(cellKey(parsed.x, parsed.y));
                    memory.removeAgentPercept(parsed.x, parsed.y);
                    memory.getMemoryGrid().set(parsed.x, parsed.y, null);
                    cellObservedAt.remove(cellKey(parsed.x, parsed.y));
                    break;
                case ACTION_FILL_HOLE:
                    claimedTargets.add(cellKey(parsed.x, parsed.y));
                    memory.removeAgentPercept(parsed.x, parsed.y);
                    memory.getMemoryGrid().set(parsed.x, parsed.y, null);
                    cellObservedAt.remove(cellKey(parsed.x, parsed.y));
                    break;
                case OBS_FUEL_ONCE:
                    if (fuelStationPosition == null) {
                        fuelStationPosition = new Int2D(parsed.x, parsed.y);
                        memory.getMemoryGrid().set(parsed.x, parsed.y,
                                new TWFuelStation(parsed.x, parsed.y,
                                        getEnvironment()));
                    }
                    break;
                default:
                    break;
            }
        }
    }

    private boolean isClaimed(int x, int y) {
        return claimedTargets.contains(cellKey(x, y));
    }


    // Utilities
    private static String cellKey(int x, int y) {
        return x + "," + y;
    }

    private static int manhattan(Int2D a, Int2D b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }
}
