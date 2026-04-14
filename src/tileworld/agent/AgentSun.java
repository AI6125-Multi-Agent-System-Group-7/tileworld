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
import tileworld.environment.TWTile;

/**
 * AgentSun
 *
 * Extends Group7AgentBase with:
 * - Zone coordination (ZA/ZK bootstrap, boustrophedon zone sweep)
 * - Target lock protocol (TL/TR send + receive)
 * - Hierarchical think(): bootstrap -> fuel -> immediate -> task -> explore
 * - Anti-stuck gate (greedy fallback + random move)
 */
public class AgentSun extends Group7AgentBase {

    private int stepCount;

    // Fuel thresholds
    private static final int CRITICAL_FUEL_MARGIN = 30;
    private static final int OPPORTUNISTIC_FUEL_THRESHOLD = 150;
    private static final int NEAR_STATION_DISTANCE = 15;
    private static final int DETOUR_FUEL_BUFFER = 10;
    // Memory freshness
    private static final double MEMORY_EXPIRY = computeMemoryExpiry();
    /**
     * Computes a dynamic memory expiry based on object lifetime, map size, and
     * object density.  The formula ensures that by the time the agent reaches a
     * remembered target, the object still has at least ~30% survival probability.
     * P(alive at arrival) = 1 - (memoryAge + travelDist) / lifeTime
     * Setting P >= 0.3 and solving for memoryAge gives:
     *   memoryAge <= lifeTime * 0.7 - travelDist
     * where travelDist ~ sqrt(mapArea / aliveObjects).
     */
    private static double computeMemoryExpiry() {
        double L = Parameters.lifeTime;
        double mapArea = Parameters.xDimension * Parameters.yDimension;
        double aliveObjects = (Parameters.tileMean + Parameters.holeMean) * L;
        double typicalDist = Math.sqrt(mapArea / Math.max(1, aliveObjects));
        double expiry = L * 0.7 - typicalDist;
        return Math.max(Parameters.defaultSensorRange, expiry);
    }
    // Target lock constants
    private static final int LOCK_TTL_MIN_STEPS = 8;
    private static final int LOCK_TTL_MAX_STEPS = 20;
    private static final int LOCK_RENEW_BEFORE_STEPS = 2;
    private static final String LOCK_TYPE_TILE = "TILE";
    private static final String LOCK_TYPE_HOLE = "HOLE";
    private static final double LOCK_PRIORITY_EPSILON = 1e-9;
    // Regional exploration state (fallback when no zone assigned)
    private final List<Int2D> regionalSearchPoints;
    private int currentRegionIndex;
    private Int2D currentRegionTarget;
    // Zone sweep state (used during bootstrap and zoned exploration)
    private String zoneSweepKey;
    private Int2D zoneSweepTarget;
    private int zoneSweepX;
    private int zoneSweepY;
    private boolean zoneSweepRight;
    //Cached fuel-station position 
    private Int2D fuelStationPosition;
    // Team coordination: completed targets (PK/FH)
    private final Set<String> claimedTargets;
    // Team coordination: active teammate target leases (TL)
    // cellKey -> TargetLease (owner, priority, step, expiresAtStep)
    private final Map<String, TargetLease> teammateTargetLeases;
    // Own target lock 
    private Int2D ownLockedCell;
    private String ownLockType;        // LOCK_TYPE_TILE or LOCK_TYPE_HOLE
    private double ownLockPriority;
    private long ownLockStep;
    private long ownLockExpiryStep;
    private final int ownLockTtlSteps;
    // Percept observation timestamps (for self-managed expiry)
    private final Map<String, Double> cellObservedAt;
    // Anti-stuck
    private int stuckCounter;
    private final Random rng;
    private static final TWDirection[] DIRS = {
        TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W
    };


    // Constructor
    public AgentSun(String name, int xpos, int ypos,
                    TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);

        this.regionalSearchPoints = new ArrayList<Int2D>();
        this.currentRegionIndex   = 0;
        this.currentRegionTarget  = null;

        this.zoneSweepKey    = null;
        this.zoneSweepTarget = null;
        this.zoneSweepX      = 0;
        this.zoneSweepY      = 0;
        this.zoneSweepRight  = true;

        this.fuelStationPosition = null;

        this.claimedTargets       = new HashSet<String>();
        this.teammateTargetLeases = new HashMap<String, TargetLease>();

        this.ownLockedCell     = null;
        this.ownLockType       = null;
        this.ownLockPriority   = 0.0;
        this.ownLockStep       = -1L;
        this.ownLockExpiryStep = -1L;
        this.ownLockTtlSteps   = computeLockTtl(env);

        this.cellObservedAt = new HashMap<String, Double>();

        this.stuckCounter = 0;
        this.rng = new Random(name.hashCode());

        initializeRegionalSearchPoints();
        enableZoneCoordination();  // opt-in to ZA/ZK bootstrap
    }


    // Regional search-point generation (fallback when no zone assigned)
    private void initializeRegionalSearchPoints() {
        int xDim = getEnvironment().getxDimension();
        int yDim = getEnvironment().getyDimension();
        int margin = Parameters.defaultSensorRange + 1;

        int left   = margin;
        int right  = xDim - 1 - margin;
        int top    = margin;
        int bottom = yDim - 1 - margin;
        int midX   = xDim / 2;
        int midY   = yDim / 2;

        if (xDim <= 60 && yDim <= 60) {
            regionalSearchPoints.add(new Int2D(left,  top));
            regionalSearchPoints.add(new Int2D(right, top));
            regionalSearchPoints.add(new Int2D(midX,  midY));
            regionalSearchPoints.add(new Int2D(right, bottom));
            regionalSearchPoints.add(new Int2D(left,  bottom));
        } else {
            regionalSearchPoints.add(new Int2D(left,  top));
            regionalSearchPoints.add(new Int2D(midX,  top));
            regionalSearchPoints.add(new Int2D(right, top));
            regionalSearchPoints.add(new Int2D(right, bottom));
            regionalSearchPoints.add(new Int2D(midX,  bottom));
            regionalSearchPoints.add(new Int2D(left,  bottom));
        }
    }

  
    // appendCustomMessages - TL renewal before expiry
    @Override
    protected void appendCustomMessages(List<Message> outbox) {
        long step = currentStep();
        if (hasActiveLock() && shouldRenewLock(step)) {
            outbox.add(createProtocolMessage(
                    CommType.TARGET_LOCK,
                    ownLockedCell.x,
                    ownLockedCell.y,
                    encodeTargetLockPayload(ownLockPriority, ownLockTtlSteps)));
            ownLockStep       = step;
            ownLockExpiryStep = step + ownLockTtlSteps;
        }
    }


    // act() - logging + release lock on successful PICKUP / PUTDOWN
    @Override
    protected void act(TWThought thought) {
        stepCount++;
        String action = (thought == null) ? "NULL" : thought.getAction().name();
        String dir = (thought != null && thought.getAction() == TWAction.MOVE)
                ? thought.getDirection().name()
                : "-";
        String lockInfo = hasActiveLock()
                ? (ownLockType + "@" + ownLockedCell.x + "," + ownLockedCell.y)
                : "-";

        System.out.println(String.format(
                "[Sun step %d] %s %s | pos=(%d,%d) fuel=%d score=%d carried=%d lock=%s",
                stepCount, action, dir,
                this.getX(), this.getY(),
                (int) this.getFuelLevel(),
                this.getScore(),
                this.carriedTiles.size(),
                lockInfo));

        int carriedBefore = this.carriedTiles.size();
        int scoreBefore   = this.getScore();

        super.act(thought);

        if (thought != null && thought.getAction() == TWAction.PICKUP
                && this.carriedTiles.size() > carriedBefore) {
            releaseLock("pickup_done");
        }
        if (thought != null && thought.getAction() == TWAction.PUTDOWN
                && this.getScore() > scoreBefore) {
            releaseLock("putdown_done");
        }
    }


    // think()
    @Override
    protected TWThought think() {
        long step = currentStep();
        // Housekeeping 
        refreshObservationCache();
        cleanExpiredPercepts(MEMORY_EXPIRY);
        updateFuelStationKnowledge();
        parseTeammateMessages(step);
        pruneStaleLeases(step);
        ensureActiveLockStillValid(step);
        processZoneCoordinationInThink();

        // Bootstrap: explore assigned zone until fuel station is found 
        if (!isZoneCoordinationComplete()) {
            if (fuelStationPosition != null) {
                markZoneCoordinationComplete();
            } else {
                releaseLock("zone_bootstrap");
                return ensureNotStuck(bootstrapZoneSearch(step));
            }
        }

        // Priority 1: Fuel management
        TWThought fuelDecision = handleFuelManagement();
        if (fuelDecision != null) {
            return ensureNotStuck(fuelDecision);
        }

        // Priority 2: Immediate cell action (no movement needed)
        TWThought immediateAction = handleImmediateAction();
        if (immediateAction != null) {
            stuckCounter = 0;
            return immediateAction;
        }

        // Priority 3: Task completion (collect tiles, fill holes)
        TWThought taskDecision = selectBestTask(step);
        if (taskDecision != null) {
            return ensureNotStuck(taskDecision);
        }

        // Priority 4: Regional exploration
        releaseLock("explore_no_task");
        return ensureNotStuck(exploreRegionally());
    }

  
    // Bootstrap zone search (before fuel station is known)
    private TWThought bootstrapZoneSearch(long step) {
        if (step == 0L) {
            return waitThought(); // wait one step to collect SS broadcasts
        }

        ZoneAssignment zone = getZoneAssignment();
        if (zone == null) {
            return waitThought(); // waiting for ZA from manager
        }

        if (!zone.contains(this.getX(), this.getY())) {
            resetZoneSweepIfNeeded(zone);
            clearPlan();
            return stepToward(zone.center());
        }

        return exploreWithinZone(zone);
    }


    // Zone-based boustrophedon sweep
    private TWThought exploreWithinZone(ZoneAssignment zone) {
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
        String key = zone.epoch + ":" + zone.x1 + ":" + zone.y1
                + ":" + zone.x2 + ":" + zone.y2;
        if (key.equals(zoneSweepKey)) {
            return;
        }
        zoneSweepKey    = key;
        zoneSweepTarget = null;
        zoneSweepX      = zone.x1;
        zoneSweepY      = zone.y1;
        zoneSweepRight  = true;
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
            // Force one boundary strip scan instead of wrapping immediately.
            zoneSweepY = zone.y2;
        }

        return new Int2D(zoneSweepX, zoneSweepY);
    }

    private boolean atPosition(Int2D target) {
        return target != null && this.getX() == target.x && this.getY() == target.y;
    }


    // Anti-stuck gate (wraps all movement decisions)
    private TWThought ensureNotStuck(TWThought thought) {
        if (!isWaitThought(thought)) {
            stuckCounter = 0;
            return thought;
        }

        stuckCounter++;
        clearPlan();

        if (currentTarget != null) {
            TWThought greedy = greedyStepToward(currentTarget);
            if (greedy != null) {
                stuckCounter = 0;
                return greedy;
            }
        }

        TWThought random = tryRandomMove();
        if (random != null) {
            return random;
        }

        return thought;
    }


    // Movement helpers
    private TWThought greedyStepToward(Int2D target) {
        int dx = target.x - getX();
        int dy = target.y - getY();

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
        // Always top up when standing on station and tank is not full.
        if (getEnvironment().inFuelStation(this)
                && getFuelLevel() < Parameters.defaultFuelLevel) {
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }
        if (isCriticalFuelLevel()) {
            releaseLock("refuel_critical");
            return navigateToFuelStation();
        }
        if (shouldRefuelOpportunistically()) {
            releaseLock("refuel_opportunistic");
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
        return getFuelLevel() < OPPORTUNISTIC_FUEL_THRESHOLD
                && dist < NEAR_STATION_DISTANCE;
    }

    private TWThought navigateToFuelStation() {
        if (fuelStationPosition != null) {
            TWThought step = stepToward(fuelStationPosition);
            if (isWaitThought(step)) {
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

  
    // Task selection
    private TWThought selectBestTask(long step) {
        if (hasTile()) {
            TWThought holeStep = findAndGoToHole(step);
            if (holeStep != null) {
                return holeStep;
            }
        }

        if (carriedTiles.size() < 3) {
            TWThought tileStep = findAndGoToTile(step);
            if (tileStep != null) {
                return tileStep;
            }
        }

        releaseLock("no_task_found");
        return null;
    }

    /**
     * Returns true if our locked target of the given type is no longer present
     * in sensor range (objects that have disappeared while we can see them).
     */
    private boolean isOwnLockTargetGone(String lockType) {
        if (!hasActiveLock() || !lockType.equals(ownLockType)) {
            return false;
        }
        int tx = ownLockedCell.x;
        int ty = ownLockedCell.y;
        // Only check if within sensor range; outside sensor range we trust memory
        if (Math.abs(tx - getX()) <= Parameters.defaultSensorRange
                && Math.abs(ty - getY()) <= Parameters.defaultSensorRange) {
            Object obj = getEnvironment().getObjectGrid().get(tx, ty);
            if (LOCK_TYPE_TILE.equals(lockType) && !(obj instanceof TWTile)) {
                return true;
            }
            if (LOCK_TYPE_HOLE.equals(lockType) && !(obj instanceof TWHole)) {
                return true;
            }
        }
        return false;
    }

    private TWThought findAndGoToHole(long step) {
        // Sticky lock: if we already have a hole lock, keep navigating to it
        if (hasActiveLock() && LOCK_TYPE_HOLE.equals(ownLockType)) {
            if (isOwnLockTargetGone(LOCK_TYPE_HOLE)) {
                releaseLock("hole_gone");
            } else {
                TWThought s = stepToward(ownLockedCell);
                if (!isWaitThought(s)) return s;
                TWThought greedy = greedyStepToward(ownLockedCell);
                if (greedy != null) return greedy;
                // Can't reach it - release and search for another
                releaseLock("hole_unreachable");
            }
        }

        // No hole lock yet - search for one (only acquire a new lock, don't switch)
        TWHole visibleHole = closestVisibleHole();
        if (visibleHole != null && !isClaimed(visibleHole.getX(), visibleHole.getY())) {
            Int2D target = new Int2D(visibleHole.getX(), visibleHole.getY());
            if (tryAcquireLock(target.x, target.y, LOCK_TYPE_HOLE, step)) {
                TWThought s = stepToward(target);
                if (!isWaitThought(s)) return s;
                TWThought greedy = greedyStepToward(target);
                if (greedy != null) return greedy;
                releaseLock("visible_hole_unreachable");
            }
        }

        TWHole knownHole = memory.getNearbyHole(getX(), getY(), MEMORY_EXPIRY);
        if (knownHole != null && !isClaimed(knownHole.getX(), knownHole.getY())) {
            Int2D target = new Int2D(knownHole.getX(), knownHole.getY());
            if (tryAcquireLock(target.x, target.y, LOCK_TYPE_HOLE, step)) {
                TWThought s = stepToward(target);
                if (!isWaitThought(s)) return s;
                TWThought greedy = greedyStepToward(target);
                if (greedy != null) return greedy;
                releaseLock("memory_hole_unreachable");
            }
        }

        return null;
    }

    private TWThought findAndGoToTile(long step) {
        // Sticky lock: if we already have a tile lock, keep navigating to it
        if (hasActiveLock() && LOCK_TYPE_TILE.equals(ownLockType)) {
            if (isOwnLockTargetGone(LOCK_TYPE_TILE)) {
                releaseLock("tile_gone");
            } else if (!isFuelSafeForDetour(ownLockedCell.x, ownLockedCell.y)) {
                releaseLock("tile_fuel_unsafe");
            } else {
                TWThought s = stepToward(ownLockedCell);
                if (!isWaitThought(s)) return s;
                TWThought greedy = greedyStepToward(ownLockedCell);
                if (greedy != null) return greedy;
                releaseLock("tile_unreachable");
            }
        }

        // No tile lock yet - search for one
        TWTile visibleTile = closestVisibleTile();
        if (visibleTile != null
                && !isClaimed(visibleTile.getX(), visibleTile.getY())
                && isFuelSafeForDetour(visibleTile.getX(), visibleTile.getY())) {
            Int2D target = new Int2D(visibleTile.getX(), visibleTile.getY());
            if (tryAcquireLock(target.x, target.y, LOCK_TYPE_TILE, step)) {
                TWThought s = stepToward(target);
                if (!isWaitThought(s)) return s;
                TWThought greedy = greedyStepToward(target);
                if (greedy != null) return greedy;
                releaseLock("visible_tile_unreachable");
            }
        }

        TWTile knownTile = memory.getNearbyTile(getX(), getY(), MEMORY_EXPIRY);
        if (knownTile != null
                && !isClaimed(knownTile.getX(), knownTile.getY())
                && isFuelSafeForDetour(knownTile.getX(), knownTile.getY())) {
            Int2D target = new Int2D(knownTile.getX(), knownTile.getY());
            if (tryAcquireLock(target.x, target.y, LOCK_TYPE_TILE, step)) {
                TWThought s = stepToward(target);
                if (!isWaitThought(s)) return s;
                TWThought greedy = greedyStepToward(target);
                if (greedy != null) return greedy;
                releaseLock("memory_tile_unreachable");
            }
        }

        return null;
    }

    private boolean isFuelSafeForDetour(int targetX, int targetY) {
        if (fuelStationPosition == null) {
            return manhattan(new Int2D(getX(), getY()),
                             new Int2D(targetX, targetY)) < CRITICAL_FUEL_MARGIN;
        }
        int distToTarget        = manhattan(new Int2D(getX(), getY()),
                                            new Int2D(targetX, targetY));
        int distTargetToStation = manhattan(new Int2D(targetX, targetY),
                                            fuelStationPosition);
        return getFuelLevel() > (distToTarget + distTargetToStation + DETOUR_FUEL_BUFFER);
    }


    // Regional exploration (zone-aware)
    private TWThought exploreRegionally() {
        // Prefer assigned zone when available
        ZoneAssignment zone = getZoneAssignment();
        if (zone != null) {
            return exploreWithinZone(zone);
        }

        // Fallback: fixed anchor points
        if (regionalSearchPoints.isEmpty()) {
            return exploreZigZag();
        }

        if (currentRegionTarget == null) {
            currentRegionTarget = addJitter(regionalSearchPoints.get(currentRegionIndex));
        }

        if (manhattan(new Int2D(getX(), getY()), currentRegionTarget)
                <= Parameters.defaultSensorRange) {
            advanceRegion();
        }

        TWThought step = stepToward(currentRegionTarget);
        if (!isWaitThought(step)) {
            return step;
        }

        TWThought greedy = greedyStepToward(currentRegionTarget);
        if (greedy != null) {
            return greedy;
        }

        clearPlan();
        advanceRegion();
        return exploreZigZag();
    }

    private void advanceRegion() {
        currentRegionIndex  = (currentRegionIndex + 1) % regionalSearchPoints.size();
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


    // Teammate message parsing
    private void parseTeammateMessages(long step) {
        claimedTargets.clear();

        List<Message> inbox = new ArrayList<Message>(getEnvironment().getMessages());
        for (Message msg : inbox) {
            ParsedMessage parsed = parseProtocolMessage(msg);
            if (parsed == null) continue;
            if (parsed.from != null && parsed.from.equals(getName())) continue;
            if (!isMessageForMe(parsed)) continue;

            switch (parsed.type) {
                case ACTION_PICKUP_TILE:
                    claimedTargets.add(cellKey(parsed.x, parsed.y));
                    memory.removeAgentPercept(parsed.x, parsed.y);
                    memory.getMemoryGrid().set(parsed.x, parsed.y, null);
                    break;

                case ACTION_FILL_HOLE:
                    claimedTargets.add(cellKey(parsed.x, parsed.y));
                    memory.removeAgentPercept(parsed.x, parsed.y);
                    memory.getMemoryGrid().set(parsed.x, parsed.y, null);
                    break;

                case OBS_FUEL_ONCE:
                    if (fuelStationPosition == null) {
                        fuelStationPosition = new Int2D(parsed.x, parsed.y);
                        memory.getMemoryGrid().set(parsed.x, parsed.y,
                                new TWFuelStation(parsed.x, parsed.y,
                                        getEnvironment()));
                    }
                    break;

                case TARGET_LOCK:
                    handleTeammateLock(parsed);
                    break;

                case TARGET_RELEASE:
                    handleTeammateTargetRelease(parsed);
                    break;

                default:
                    break;
            }
        }
    }

    private void handleTeammateLock(ParsedMessage parsed) {
        Double priority = parseTargetLockPriority(parsed.payload);
        if (priority == null) return;
        Integer ttl = parseTargetLockTtl(parsed.payload);
        int ttlSteps = (ttl == null) ? ownLockTtlSteps : ttl.intValue();

        String key = cellKey(parsed.x, parsed.y);
        TargetLease incoming = new TargetLease(parsed.from, priority, parsed.step, ttlSteps);
        TargetLease existing = teammateTargetLeases.get(key);

        if (existing == null) {
            teammateTargetLeases.put(key, incoming);
        } else if (existing.owner.equals(incoming.owner)) {
            // Same teammate renewing — keep the newer step
            if (incoming.step >= existing.step) {
                teammateTargetLeases.put(key, incoming);
            }
        } else {
            // Different teammates competing for same cell — keep the preferred one
            if (isPreferredLock(incoming.owner, incoming.priority, incoming.step,
                                existing.owner, existing.priority, existing.step)) {
                teammateTargetLeases.put(key, incoming);
            }
        }
        // Priority arbitration is handled centrally in ensureActiveLockStillValid()
    }

    private void handleTeammateTargetRelease(ParsedMessage parsed) {
        String key = cellKey(parsed.x, parsed.y);
        TargetLease existing = teammateTargetLeases.get(key);
        // Only remove if the release comes from the lease owner
        if (existing != null && existing.owner.equals(parsed.from)) {
            teammateTargetLeases.remove(key);
        }
    }

    private void pruneStaleLeases(long step) {
        List<String> stale = new ArrayList<String>();
        for (Map.Entry<String, TargetLease> entry : teammateTargetLeases.entrySet()) {
            if (step > entry.getValue().expiresAtStep) {
                stale.add(entry.getKey());
            }
        }
        for (String key : stale) {
            teammateTargetLeases.remove(key);
        }
    }

    private boolean isClaimed(int x, int y) {
        String key = cellKey(x, y);
        if (claimedTargets.contains(key)) {
            return true;
        }
        TargetLease lease = teammateTargetLeases.get(key);
        return lease != null && currentStep() <= lease.expiresAtStep;
    }


    // Self-managed percept expiry (replaces TWAgentWorkingMemory.cleanExpiredPercepts)
    /**
     * Records the current simulation time for every tile, hole, or obstacle
     * visible in sensor range. Called each step before cleanExpiredPercepts().
     */
    private void refreshObservationCache() {
        double now = getEnvironment().schedule.getTime();
        int range = Parameters.defaultSensorRange;
        for (int dx = -range; dx <= range; dx++) {
            for (int dy = -range; dy <= range; dy++) {
                int x = getX() + dx;
                int y = getY() + dy;
                if (!getEnvironment().isInBounds(x, y)) {
                    continue;
                }
                Object obj = getEnvironment().getObjectGrid().get(x, y);
                if (obj instanceof TWTile
                        || obj instanceof TWHole
                        || obj instanceof tileworld.environment.TWObstacle) {
                    cellObservedAt.put(cellKey(x, y), now);
                } else if (obj == null) {
                    // Cell is now empty — evict any stale tile/hole percept immediately
                    String key = cellKey(x, y);
                    if (cellObservedAt.remove(key) != null) {
                        memory.removeAgentPercept(x, y);
                        memory.getMemoryGrid().set(x, y, null);
                    }
                }
            }
        }
    }

    /**
     * Removes percepts from working memory whose last observed time is older
     * than maxAge simulation steps. Fuel stations are never expired.
     */
    private void cleanExpiredPercepts(double maxAge) {
        double now = getEnvironment().schedule.getTime();
        List<String> expired = new ArrayList<String>();
        for (Map.Entry<String, Double> entry : cellObservedAt.entrySet()) {
            if (now - entry.getValue() > maxAge) {
                expired.add(entry.getKey());
            }
        }
        for (String key : expired) {
            cellObservedAt.remove(key);
            String[] parts = key.split(",");
            int x = Integer.parseInt(parts[0]);
            int y = Integer.parseInt(parts[1]);
            memory.removeAgentPercept(x, y);
            memory.getMemoryGrid().set(x, y, null);
        }
    }


    // Own target lock validation (called every step after message parsing)
    /**
     * Validates the current lock each step:
     * 1. Release if our own TTL has expired.
     * 2. Release if a teammate now holds a higher-priority lease on the same cell.
     * 3. Release if the target has visibly disappeared from sensor range.
     */
    private void ensureActiveLockStillValid(long step) {
        if (!hasActiveLock()) {
            return;
        }

        // 1. Own TTL expired
        if (step > ownLockExpiryStep) {
            releaseLock("own_lease_expired");
            return;
        }

        // 2. Priority arbitration: yield only if teammate has higher priority
        TargetLease teammate = teammateTargetLeases.get(cellKey(ownLockedCell.x, ownLockedCell.y));
        if (teammate != null) {
            if (!isPreferredLock(getName(), ownLockPriority, ownLockStep,
                                 teammate.owner, teammate.priority, teammate.step)) {
                releaseLock("lost_priority");
                return;
            }
        }

        // 3. Target visibly gone from sensor range
        if (isOwnLockTargetGone(ownLockType)) {
            releaseLock("target_changed_or_missing");
        }
    }


    // Own target lock management
    private boolean hasActiveLock() {
        return ownLockedCell != null && ownLockType != null;
    }

    private boolean isOwnLockAt(int x, int y, String type) {
        return hasActiveLock()
                && ownLockedCell.x == x
                && ownLockedCell.y == y
                && ownLockType.equals(type);
    }

    private boolean shouldRenewLock(long step) {
        if (!hasActiveLock()) {
            return false;
        }
        if (ownLockExpiryStep < 0) {
            return true;
        }
        return step >= (ownLockExpiryStep - LOCK_RENEW_BEFORE_STEPS);
    }

    /**
     * Acquire or keep a target lock on (x, y, type).
     * Returns false only if a teammate with strictly higher priority already
     * holds a lease on the same cell.
     */
    private boolean tryAcquireLock(int x, int y, String type, long step) {
        // Already holding this exact lock - keep it
        if (isOwnLockAt(x, y, type)) {
            return true;
        }

        // Yield only if a teammate with higher priority holds this cell
        double priority = computeLockPriority(x, y, type, step);
        if (isBlockedByPreferredTeammateLock(x, y, priority, step)) {
            return false;
        }

        // Acquire: release old lock first, then claim new one
        releaseLock("switch_target");
        ownLockedCell     = new Int2D(x, y);
        ownLockType       = type;
        ownLockPriority   = priority;
        ownLockStep       = step;
        ownLockExpiryStep = step + ownLockTtlSteps;
        publishProtocolMessage(
                CommType.TARGET_LOCK,
                x, y,
                encodeTargetLockPayload(ownLockPriority, ownLockTtlSteps));
        return true;
    }

    private void releaseLock(String reason) {
        if (!hasActiveLock()) {
            return;
        }
        publishProtocolMessage(
                CommType.TARGET_RELEASE,
                ownLockedCell.x,
                ownLockedCell.y,
                (reason == null) ? "" : reason);
        ownLockedCell     = null;
        ownLockType       = null;
        ownLockPriority   = 0.0;
        ownLockStep       = -1L;
        ownLockExpiryStep = -1L;
    }

    /** Pseudo-random priority in [0,1), stable across agents for the same (x, y, type, step). */
    private double computeLockPriority(int x, int y, String type, long step) {
        int typeInt = LOCK_TYPE_TILE.equals(type) ? 0 : 1;
        long seed = 1469598103934665603L;
        seed ^= (long) getName().hashCode();
        seed *= 1099511628211L;
        seed ^= (long) typeInt;
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
        return (positive % 1_000_000L) / 1_000_000.0;
    }

    /**
     * Returns true if ownerA's lock is preferred over ownerB's lock.
     * Higher priority wins; tie-breaks: earlier step wins; then lexicographic name.
     */
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

    /**
     * Returns true if a teammate holds a lease on (x, y) that beats our
     * candidate priority — meaning we should NOT acquire this cell.
     */
    private boolean isBlockedByPreferredTeammateLock(int x, int y, double myPriority, long myStep) {
        TargetLease teammate = teammateTargetLeases.get(cellKey(x, y));
        if (teammate == null) {
            return false;
        }
        return !isPreferredLock(getName(), myPriority, myStep,
                                teammate.owner, teammate.priority, teammate.step);
    }


    // isWaitThought helper
    private boolean isWaitThought(TWThought thought) {
        return thought != null
                && thought.getAction() == TWAction.MOVE
                && thought.getDirection() == TWDirection.Z;
    }


    // Utilities
    private static int computeLockTtl(TWEnvironment env) {
        int span = Math.max(1, env.getxDimension() + env.getyDimension());
        int ttl  = span / 6;
        if (ttl < LOCK_TTL_MIN_STEPS) return LOCK_TTL_MIN_STEPS;
        if (ttl > LOCK_TTL_MAX_STEPS) return LOCK_TTL_MAX_STEPS;
        return ttl;
    }

    private long currentStep() {
        return this.getEnvironment().schedule.getSteps();
    }

    private static String cellKey(int x, int y) {
        return x + "," + y;
    }

    private static int manhattan(Int2D a, Int2D b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }


    // TargetLease inner class
    /**
     * Represents a teammate's active claim on a cell.
     * Stores enough information to perform priority arbitration.
     */
    private static class TargetLease {
        final String owner;
        final double priority;
        final long step;
        final long expiresAtStep;

        TargetLease(String owner, double priority, long step, int ttlSteps) {
            this.owner       = owner;
            this.priority    = priority;
            this.step        = step;
            this.expiresAtStep = step + Math.max(1, ttlSteps);
        }
    }
}
