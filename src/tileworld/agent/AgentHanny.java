package tileworld.agent;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import sim.field.grid.ObjectGrid2D;
import sim.util.Int2D;
import tileworld.agent.classdefines.MemoryObjectType;
import tileworld.agent.classdefines.MemorySideCard;
import tileworld.agent.classdefines.MemorySideCardEntry;
import tileworld.agent.utils.HazardLearningUtils;
import tileworld.agent.utils.RuntimeFileLogger;
import tileworld.agent.utils.SensorSnapshotCodec;
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
 * AgentHanny by HumanHanny
 *
 * This implementation keeps the original light-weight policy style but adds a
 * modular hazard-aware planning layer:
 *
 * 1) MemorySideCard augments TWAgentWorkingMemory and is always synchronized.
 * 2) Three hazards are learned online (tile/hole/obstacle).
 * 3) Teammate messages continuously refresh memory and learning signals.
 * 4) Candidate recall uses top-20 with +5 buffer and lazy deletion.
 */
public class AgentHanny extends Group7AgentBase {
    private static final int FUEL_MARGIN = 32;  // margin estimation error allowed for reaching fuel station
    private static final int OBSERVATION_INTERVAL_STEPS = 3;
    private static final int RECALL_TOP_K = 20;
    private static final int RECALL_BUFFER = 5;
    private static final double P_THETA = 0.01;
    private static final double EPSILON = 1e-8;
    private static final double LEARNING_RATE_MATCH_BASE = 0.003;
    private static final double LEARNING_RATE_MISMATCH_BASE = 0.012;
    private static final double LEARNING_RATE_MATCH_MIN = 0.0008;
    private static final double LEARNING_RATE_MISMATCH_MIN = 0.0025;
    private static final double MATCH_TIME_DECAY = 0.0009;
    private static final double MISMATCH_TIME_DECAY = 0.00035;
    private static final int MISMATCH_BURST_THRESHOLD = 2;
    private static final double EVENT_DECAY_FACTOR = 0.90;
    private static final double EVENT_RECOVERY_FACTOR = 1.02;
    private static final double EVENT_MULTIPLIER_MIN = 0.45;
    private static final double HAZARD_MIN = 1e-3;
    private static final double HAZARD_MAX = 2.0;
    private static final double MAX_ABS_HAZARD_STEP = 0.05;
    private static final int PLAN_TILE_RECALL_LIMIT = 7;
    private static final int PLAN_HOLE_RECALL_LIMIT = 7;
    private static final int PLAN_MAX_DEPTH = 6;
    private static final double PLAN_PICKUP_UTILITY = 0.20;
    private static final double PLAN_FILL_UTILITY = 1.00;
    private static final double PLAN_DISTANCE_COST = 0.07;
    private static final double PLAN_FUEL_PENALTY = 10.0;
    private static final int EXPLORE_GRID_TARGET_CELL_SIZE = 15;
    private static final int EXPLORE_MAX_SECTOR_CENTER_DISTANCE = 100;
    private static final long TEAMMATE_POSITION_TTL = 6L;
    private static final int EXPLORE_SECTOR_SWITCH_COOLDOWN = 4;
    private static final double EXPLORE_TEAMMATE_PENALTY = 1.80;
    private static final double EXPLORE_COVERAGE_PENALTY = 1.20;
    private static final double EXPLORE_DISTANCE_PENALTY = 0.03;
    private static final boolean ENABLE_STEP_LOG = false;
    private static final boolean ENABLE_RUNTIME_LOG = true;
    private static final int LOCK_TTL_MIN_STEPS = 8;
    private static final int LOCK_TTL_MAX_STEPS = 20;
    private static final int LOCK_RENEW_BEFORE_STEPS = 2;
    private static final double LOCK_PRIORITY_EPSILON = 1e-9;
    private static final String LOG_DIR = "src/tileworld/agent/runtime";

    private int stepCount;

    private final MemorySideCard sideCard;
    private final RuntimeFileLogger runtimeLogger;

    // Tracks whether a cell was explicitly observed empty at a specific step.
    private final Map<String, Long> selfEmptyObservedStep;
    private final Map<String, Long> teammateEmptyObservedStep;
    private final Map<String, TargetLease> teammateTargetLeases;
    private final Map<String, TeammatePosition> teammatePositions;
    private final int ownTargetLockTtlSteps;

    // Owned target lock (renewed shortly before expiry).
    private Int2D lockedTargetCell;
    private MemoryObjectType lockedTargetType;
    private double lockedTargetPriority;
    private long lockedTargetStep;
    private long lockedTargetExpiryStep;

    // Bootstrap zone-search state before fuel station is known globally.
    private String zoneSweepKey;
    private Int2D zoneSweepTarget;
    private int zoneSweepX;
    private int zoneSweepY;
    private boolean zoneSweepRight;

    // Adaptive exploration over dynamic n x n sectors.
    private final SectorRect[] exploreSectors;
    private final boolean[][] exploredEver;
    private final int[] exploredCellsPerSector;
    private int currentExploreSectorId;
    private long lastExploreSectorSwitchStep;
    private String exploreSweepKey;
    private Int2D exploreSweepTarget;
    private int exploreSweepX;
    private int exploreSweepY;
    private boolean exploreSweepRight;

    // Rolling plan selected by EU over multi-object sequences.
    private PlanCandidate activePlan;
    private int activePlanIndex;

    // Online learned hazards (lambda) per object type.
    private double hazardTile;
    private double hazardHole;
    private double hazardObstacle;

    // Per-step aggregated gradient buckets and schedule state.
    private final EnumMap<MemoryObjectType, TypeGradientBucket> gradientBuckets;
    private final EnumMap<MemoryObjectType, Double> mismatchEventMultipliers;

    public AgentHanny(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
        this.sideCard = new MemorySideCard();
        this.runtimeLogger = new RuntimeFileLogger(buildRuntimeLogPath(name));
        this.selfEmptyObservedStep = new HashMap<String, Long>();
        this.teammateEmptyObservedStep = new HashMap<String, Long>();
        this.teammateTargetLeases = new HashMap<String, TargetLease>();
        this.teammatePositions = new HashMap<String, TeammatePosition>();
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
        int exploreGridSplit = computeExploreGridSplit(env.getxDimension(), env.getyDimension());
        this.exploreSectors = buildExploreSectors(env.getxDimension(), env.getyDimension(), exploreGridSplit);
        this.exploredEver = new boolean[env.getxDimension()][env.getyDimension()];
        this.exploredCellsPerSector = new int[this.exploreSectors.length];
        this.currentExploreSectorId = -1;
        this.lastExploreSectorSwitchStep = Long.MIN_VALUE / 4;
        this.exploreSweepKey = null;
        this.exploreSweepTarget = null;
        this.exploreSweepX = 0;
        this.exploreSweepY = 0;
        this.exploreSweepRight = true;
        this.activePlan = null;
        this.activePlanIndex = 0;

        this.hazardTile = 0.06;
        this.hazardHole = 0.06;
        this.hazardObstacle = 0.04;

        this.gradientBuckets = new EnumMap<MemoryObjectType, TypeGradientBucket>(MemoryObjectType.class);
        this.mismatchEventMultipliers = new EnumMap<MemoryObjectType, Double>(MemoryObjectType.class);
        for (MemoryObjectType type : MemoryObjectType.values()) {
            this.gradientBuckets.put(type, new TypeGradientBucket());
            this.mismatchEventMultipliers.put(type, 1.0);
        }

        enableZoneCoordination();
    }

    @Override
    protected void appendCustomMessages(List<Message> outbox) {
        // Renew only when lease is close to expiry to reduce traffic.
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

    @Override
    protected TWThought think() {
        long step = currentStep();
        resetGradientBuckets();

        rememberFuelStationsInSensorRange();
        processIncomingMessages(step);
        processZoneCoordinationInThink();
        pruneStaleTeammateTargetLeases(step);
        pruneStaleTeammatePositions(step);
        ensureActiveLockStillValid(step);
        synchronizeSideCardWithCurrentObservation(step);
        applyAggregatedHazardUpdates(step);
        pruneEmptyObservationCaches(step);

        if (!isZoneCoordinationComplete()) {
            if (findFuelStationInMemory() != null) {
                markZoneCoordinationComplete();
            } else {
                releaseOwnTargetLock("zone_bootstrap");
                return bootstrapZoneSearch(step);
            }
        }

        // If we are already on the fuel station and low, refill immediately.
        if (this.getEnvironment().inFuelStation(this) && shouldRefuel(FUEL_MARGIN)) {
            releaseOwnTargetLock("refuel_now");
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }

        // If fuel is getting spicy, go find the station.
        if (shouldRefuel(FUEL_MARGIN)) {
            releaseOwnTargetLock("refuel_trip");
            TWFuelStation station = findFuelStationInMemory();
            if (station != null) {
                return stepToward(new Int2D(station.getX(), station.getY()));
            }
            // No station in memory? Wander until we bump into one.
            return explore();
        }

        // Immediate actions on our current cell.
        TWEntity here = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
        if (here instanceof TWHole && this.hasTile()) {
            if (!isOwnLockAt(MemoryObjectType.HOLE, this.getX(), this.getY())) {
                releaseOwnTargetLock("putdown_other_target");
            }
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (here instanceof TWTile && this.carriedTiles.size() < 3) {
            if (!isOwnLockAt(MemoryObjectType.TILE, this.getX(), this.getY())) {
                releaseOwnTargetLock("pickup_other_target");
            }
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        // Plan over multiple objects using expected utility (EU), then execute first step.
        if (shouldReplan(step)) {
            releaseOwnTargetLock("replan_before_new_plan");
            activePlan = buildBestPlan(step);
            activePlanIndex = 0;
        }

        PlanTarget current = getCurrentPlanTarget();
        if (current != null) {
            if (tryAcquireOrKeepTargetLock(current.type, current.x, current.y, step)) {
                return stepToward(new Int2D(current.x, current.y));
            }
            // Could not lock current target due teammate contention, force re-plan.
            clearActivePlan("lock_conflict_replan");
            activePlan = buildBestPlan(step);
            activePlanIndex = 0;
            current = getCurrentPlanTarget();
            if (current != null && tryAcquireOrKeepTargetLock(current.type, current.x, current.y, step)) {
                return stepToward(new Int2D(current.x, current.y));
            }
        }

        // Otherwise, explore like 'Man vs Wild' host Bear Grylls.
        releaseOwnTargetLock("explore_no_candidate");
        return explore();
    }

    @Override
    protected void act(TWThought thought) {
        stepCount++;
        String action = (thought == null) ? "NULL" : thought.getAction().name();
        String dir = (thought != null && thought.getAction() == TWAction.MOVE)
                ? thought.getDirection().name()
                : "-";

        int actionX = this.getX();
        int actionY = this.getY();
        int carriedBefore = this.carriedTiles.size();
        int scoreBefore = this.getScore();

        if (ENABLE_STEP_LOG) {
            System.out.println(String.format(
                    "[Hanny step %d] %s %s | pos=(%d,%d) fuel=%d score=%d",
                    stepCount,
                    action,
                    dir,
                    this.getX(),
                    this.getY(),
                    (int) this.getFuelLevel(),
                    this.getScore()));
        }
        super.act(thought);

        // Keep sidecard and working memory in sync with own successful actions.
        if (thought != null && thought.getAction() == TWAction.PICKUP && this.carriedTiles.size() > carriedBefore) {
            advancePlanAfterAction(MemoryObjectType.TILE, actionX, actionY);
            removeRecordAndSync(MemoryObjectType.TILE, actionX, actionY);
        }
        if (thought != null && thought.getAction() == TWAction.PUTDOWN && this.getScore() > scoreBefore) {
            advancePlanAfterAction(MemoryObjectType.HOLE, actionX, actionY);
            removeRecordAndSync(MemoryObjectType.HOLE, actionX, actionY);
        }
    }

    private TWThought explore() {
        return exploreAdaptiveNineGrid(currentStep());
    }

    private TWThought bootstrapZoneSearch(long step) {
        // Keep round-0 behavior deterministic: everyone pauses once to collect SS.
        if (step == 0L) {
            return waitThought();
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
            // Force one boundary strip scan instead of wrapping immediately.
            zoneSweepY = zone.y2;
        }

        return new Int2D(zoneSweepX, zoneSweepY);
    }

    private TWThought exploreAdaptiveNineGrid(long step) {
        if (exploreSectors.length == 0) {
            return exploreZigZag();
        }

        int bestSector = chooseExploreSector(step);
        if (bestSector < 0) {
            return exploreZigZag();
        }

        if (currentExploreSectorId < 0) {
            setCurrentExploreSector(bestSector, step);
        } else if (bestSector != currentExploreSectorId) {
            long elapsed = step - lastExploreSectorSwitchStep;
            double currentCoverage = sectorCoverage(currentExploreSectorId);
            if (elapsed >= EXPLORE_SECTOR_SWITCH_COOLDOWN || currentCoverage >= 0.92) {
                setCurrentExploreSector(bestSector, step);
            }
        }

        SectorRect sector = exploreSectors[currentExploreSectorId];
        resetExploreSweepIfNeeded(sector);

        if (!sector.contains(this.getX(), this.getY())) {
            clearPlan();
            return stepToward(sector.center());
        }

        for (int attempts = 0; attempts < 6; attempts++) {
            if (exploreSweepTarget == null || atPosition(exploreSweepTarget)) {
                exploreSweepTarget = nextExploreSweepTarget(sector);
            }

            TWThought thought = stepToward(exploreSweepTarget);
            if (!isWaitThought(thought)) {
                return thought;
            }

            exploreSweepTarget = nextExploreSweepTarget(sector);
            clearPlan();
        }

        return waitThought();
    }

    private void setCurrentExploreSector(int sectorId, long step) {
        currentExploreSectorId = sectorId;
        lastExploreSectorSwitchStep = step;
        exploreSweepKey = null;
        exploreSweepTarget = null;
    }

    private int chooseExploreSector(long step) {
        int bestId = -1;
        double bestScore = -Double.MAX_VALUE;

        for (int i = 0; i < exploreSectors.length; i++) {
            SectorRect sector = exploreSectors[i];
            if (sector.area() <= 0) {
                continue;
            }

            int teammates = teammateCountInSector(sector, step);
            double coverage = sectorCoverage(i);
            Int2D center = sector.center();
            int dist = manhattan(this.getX(), this.getY(), center.x, center.y);
            if (dist > EXPLORE_MAX_SECTOR_CENTER_DISTANCE) {
                continue;
            }

            double score = 0.0;
            score -= EXPLORE_TEAMMATE_PENALTY * teammates;
            score -= EXPLORE_COVERAGE_PENALTY * coverage;
            score -= EXPLORE_DISTANCE_PENALTY * dist;

            if (score > bestScore) {
                bestScore = score;
                bestId = i;
            }
        }

        return bestId;
    }

    private int teammateCountInSector(SectorRect sector, long step) {
        int count = 0;
        for (TeammatePosition pos : teammatePositions.values()) {
            if ((step - pos.step) > TEAMMATE_POSITION_TTL) {
                continue;
            }
            if (sector.contains(pos.x, pos.y)) {
                count += 1;
            }
        }
        return count;
    }

    private double sectorCoverage(int sectorId) {
        if (sectorId < 0 || sectorId >= exploreSectors.length) {
            return 1.0;
        }
        SectorRect sector = exploreSectors[sectorId];
        int area = sector.area();
        if (area <= 0) {
            return 1.0;
        }
        return exploredCellsPerSector[sectorId] / (double) area;
    }

    private void resetExploreSweepIfNeeded(SectorRect sector) {
        String key = sector.id + ":" + sector.x1 + ":" + sector.y1 + ":" + sector.x2 + ":" + sector.y2;
        if (key.equals(exploreSweepKey)) {
            return;
        }

        exploreSweepKey = key;
        exploreSweepTarget = null;
        exploreSweepX = sector.x1;
        exploreSweepY = sector.y1;
        exploreSweepRight = true;
    }

    private Int2D nextExploreSweepTarget(SectorRect sector) {
        int stride = Math.max(1, (Parameters.defaultSensorRange * 2) + 1);

        if (exploreSweepRight) {
            int nextX = exploreSweepX + stride;
            if (nextX <= sector.x2) {
                exploreSweepX = nextX;
            } else {
                exploreSweepX = sector.x2;
                exploreSweepY += stride;
                exploreSweepRight = false;
            }
        } else {
            int nextX = exploreSweepX - stride;
            if (nextX >= sector.x1) {
                exploreSweepX = nextX;
            } else {
                exploreSweepX = sector.x1;
                exploreSweepY += stride;
                exploreSweepRight = true;
            }
        }

        if (exploreSweepY > sector.y2) {
            // Force one boundary strip scan instead of wrapping immediately.
            exploreSweepY = sector.y2;
        }

        return new Int2D(exploreSweepX, exploreSweepY);
    }

    private void updateTeammatePosition(String name, int x, int y, long step) {
        if (name == null || name.equals(this.getName())) {
            return;
        }
        teammatePositions.put(name, new TeammatePosition(x, y, step));
    }

    private void pruneStaleTeammatePositions(long step) {
        List<String> stale = new ArrayList<String>();
        for (Map.Entry<String, TeammatePosition> entry : teammatePositions.entrySet()) {
            if ((step - entry.getValue().step) > TEAMMATE_POSITION_TTL) {
                stale.add(entry.getKey());
            }
        }
        for (String key : stale) {
            teammatePositions.remove(key);
        }
    }

    private void markCellExplored(int x, int y) {
        if (!this.getEnvironment().isInBounds(x, y)) {
            return;
        }
        if (exploredEver[x][y]) {
            return;
        }

        exploredEver[x][y] = true;
        int sectorId = sectorIdForCell(x, y);
        if (sectorId >= 0 && sectorId < exploredCellsPerSector.length) {
            exploredCellsPerSector[sectorId] += 1;
        }
    }

    private int sectorIdForCell(int x, int y) {
        for (SectorRect sector : exploreSectors) {
            if (sector.contains(x, y)) {
                return sector.id;
            }
        }
        return -1;
    }

    private int computeExploreGridSplit(int xDim, int yDim) {
        int mapSize = Math.min(xDim, yDim);
        int split = mapSize / EXPLORE_GRID_TARGET_CELL_SIZE;
        if (split < 1) {
            return 1;
        }
        return split;
    }

    private SectorRect[] buildExploreSectors(int xDim, int yDim, int split) {
        int[] xSizes = splitSizes(xDim, split);
        int[] ySizes = splitSizes(yDim, split);

        List<SectorRect> sectors = new ArrayList<SectorRect>(split * split);
        int yStart = 0;
        int id = 0;
        for (int row = 0; row < split; row++) {
            int yEnd = yStart + ySizes[row] - 1;
            int xStart = 0;
            for (int col = 0; col < split; col++) {
                int xEnd = xStart + xSizes[col] - 1;
                sectors.add(new SectorRect(id, xStart, yStart, xEnd, yEnd));
                xStart = xEnd + 1;
                id += 1;
            }
            yStart = yEnd + 1;
        }
        return sectors.toArray(new SectorRect[sectors.size()]);
    }

    private int[] splitSizes(int total, int parts) {
        int[] sizes = new int[parts];
        int base = total / parts;
        int rem = total % parts;
        for (int i = 0; i < parts; i++) {
            sizes[i] = base + ((i < rem) ? 1 : 0);
        }
        return sizes;
    }

    private boolean isWaitThought(TWThought thought) {
        return thought != null
                && thought.getAction() == TWAction.MOVE
                && thought.getDirection() == TWDirection.Z;
    }

    private boolean atPosition(Int2D target) {
        return target != null && this.getX() == target.x && this.getY() == target.y;
    }

    @Override
    protected void clearPlan() {
        super.clearPlan();
        releaseOwnTargetLock("clear_plan");
        clearActivePlan("clear_plan");
    }

    private void processIncomingMessages(long step) {
        List<Message> inbox = new ArrayList<Message>(this.getEnvironment().getMessages());
        for (Message message : inbox) {
            if (message == null) {
                continue;
            }
            String directFrom = message.getFrom();
            if (directFrom != null && directFrom.equals(this.getName())) {
                continue;
            }
            String directTo = message.getTo();
            if (directTo != null && !directTo.equals("ALL") && !directTo.equals(this.getName())) {
                continue;
            }

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
                    MemoryObjectType type = MemoryObjectType.fromCommType(parsed.type);
                    if (type != null) {
                        boolean spawnObserved = wasObservedEmptyAtStep(parsed.x, parsed.y, parsed.step - 1);
                        registerObservation(type, parsed.x, parsed.y, parsed.step, spawnObserved, "team_point_obs", null);
                    }
                    break;
                case OBS_FUEL_ONCE:
                    rememberFuelStationFromMessage(parsed.x, parsed.y);
                    break;
                case OBS_SENSOR_SNAPSHOT:
                    updateTeammatePosition(parsed.from, parsed.x, parsed.y, parsed.step);
                    processSnapshotMessage(parsed);
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

    private void processSnapshotMessage(ParsedMessage parsed) {
        List<SensorSnapshotCodec.SnapshotItem> items = SensorSnapshotCodec.decode(parsed.payload);
        for (SensorSnapshotCodec.SnapshotItem item : items) {
            if ("E".equals(item.code)) {
                teammateEmptyObservedStep.put(MemorySideCard.cellKey(item.x, item.y), parsed.step);
                handleEmptyCellEvidence(item.x, item.y, parsed.step, "team_empty");
                continue;
            }

            MemoryObjectType type = MemoryObjectType.fromSnapshotCode(item.code);
            if (type == null) {
                continue;
            }

            boolean spawnObserved = wasObservedEmptyAtStep(item.x, item.y, parsed.step - 1);
            registerObservation(type, item.x, item.y, parsed.step, spawnObserved, "team_snapshot", null);
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

        if (!sideCard.contains(lockedTargetType, lockedTargetCell.x, lockedTargetCell.y)) {
            releaseOwnTargetLock("target_not_in_sidecard");
        }
    }

    private boolean tryAcquireOrKeepTargetLock(MemoryObjectType type, int x, int y, long step) {
        if (type == null) {
            return false;
        }
        if (isOwnLockAt(type, x, y)) {
            return true;
        }

        MemorySideCardEntry entry = sideCard.get(type, x, y);
        if (entry == null) {
            return false;
        }
        return tryAcquireOrKeepTargetLock(entry, step);
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

    private boolean isOwnLockAt(MemoryObjectType type, int x, int y) {
        return hasActiveTargetLock()
                && lockedTargetType == type
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
        lockedTargetType = null;
        lockedTargetPriority = 0.0;
        lockedTargetStep = -1L;
        lockedTargetExpiryStep = -1L;
    }

    private void synchronizeSideCardWithCurrentObservation(long step) {
        ObjectGrid2D grid = this.getEnvironment().getObjectGrid();
        int range = Parameters.defaultSensorRange;
        Set<String> currentlyVisibleKeys = new HashSet<String>();

        for (int dx = -range; dx <= range; dx++) {
            for (int dy = -range; dy <= range; dy++) {
                int x = this.getX() + dx;
                int y = this.getY() + dy;
                if (!this.getEnvironment().isInBounds(x, y)) {
                    continue;
                }
                markCellExplored(x, y);

                TWEntity obj = (TWEntity) grid.get(x, y);
                if (obj == null) {
                    selfEmptyObservedStep.put(MemorySideCard.cellKey(x, y), step);
                    continue;
                }

                if (obj instanceof TWFuelStation) {
                    this.memory.getMemoryGrid().set(x, y, obj);
                    continue;
                }

                MemoryObjectType type = MemoryObjectType.fromEntity(obj);
                if (type == null) {
                    continue;
                }

                String typedKey = MemorySideCard.key(type, x, y);
                currentlyVisibleKeys.add(typedKey);

                boolean spawnObserved = false;
                if (isInnerSensorCell(x, y) && !sideCard.contains(type, x, y)) {
                    spawnObserved = wasObservedEmptyAtStep(x, y, step - 1);
                }

                registerObservation(type, x, y, step, spawnObserved, "self_snapshot", obj);
            }
        }

        // If a remembered object is inside current sensor range but absent now,
        // treat this as mismatch evidence and remove it immediately.
        List<MemorySideCardEntry> all = sideCard.listAll();
        for (MemorySideCardEntry entry : all) {
            if (!isInSensorRange(entry.getX(), entry.getY())) {
                continue;
            }
            if (currentlyVisibleKeys.contains(entry.key())) {
                continue;
            }

            TWEntity here = (TWEntity) grid.get(entry.getX(), entry.getY());
            MemoryObjectType seenType = MemoryObjectType.fromEntity(here);
            if (seenType == entry.getType()) {
                continue;
            }

            applyHazardGradient(entry, Math.max(0.0, step - entry.getLastSeen()), step, false, "self_visible_miss");
            removeRecordAndSync(entry.getType(), entry.getX(), entry.getY());
        }
    }

    private void registerObservation(
            MemoryObjectType type,
            int x,
            int y,
            double observedAt,
            boolean spawnObserved,
            String source,
            TWEntity concreteEntity) {

        MemorySideCardEntry existing = sideCard.get(type, x, y);
        double delta = (existing == null) ? 0.0 : Math.max(0.0, observedAt - existing.getLastSeen());

        MemorySideCardEntry entry = sideCard.upsert(type, x, y, observedAt, spawnObserved);
        if (existing != null) {
            applyHazardGradient(entry, delta, (long) observedAt, true, source);
        }

        TWEntity entityForMemory = (concreteEntity != null) ? concreteEntity : createGhostEntity(type, x, y, observedAt);
        syncWorkingMemorySet(entityForMemory);
    }

    private void handleEmptyCellEvidence(int x, int y, long step, String source) {
        for (MemoryObjectType type : MemoryObjectType.values()) {
            MemorySideCardEntry entry = sideCard.get(type, x, y);
            if (entry == null) {
                continue;
            }
            applyHazardGradient(entry, Math.max(0.0, step - entry.getLastSeen()), step, false, source);
            removeRecordAndSync(type, x, y);
        }
    }

    private void applyHazardGradient(
            MemorySideCardEntry entry,
            double delta,
            long step,
            boolean matched,
            String source) {

        // Spawn-observed objects follow a hard-lifetime branch in inference,
        // so we exclude them from lambda gradient updates to keep training
        // data consistent with model assumptions.
        if (entry.isSpawnObserved()) {
            return;
        }
        if (delta <= 0.0) {
            return;
        }
        if ((step - entry.getLastHazardUpdateStep()) < OBSERVATION_INTERVAL_STEPS) {
            return;
        }

        MemoryObjectType type = entry.getType();
        double currentLambda = getHazard(type);
        double weight = sourceWeight(source, matched);
        TypeGradientBucket bucket = gradientBuckets.get(type);

        if (matched) {
            bucket.matchedDeltaSum += (weight * delta);
            bucket.matchedCount += 1;
        } else {
            // Use numerically stable mismatch gradient term at current lambda.
            double denom = Math.expm1(Math.max(0.0, currentLambda * delta)) + EPSILON;
            double gradTerm = (weight * delta / denom);
            bucket.mismatchGradSum += gradTerm;
            bucket.mismatchCount += 1;
        }

        entry.setLastHazardUpdateStep(step);
    }

    private void resetGradientBuckets() {
        for (TypeGradientBucket bucket : gradientBuckets.values()) {
            bucket.reset();
        }
    }

    private void applyAggregatedHazardUpdates(long step) {
        for (MemoryObjectType type : MemoryObjectType.values()) {
            TypeGradientBucket bucket = gradientBuckets.get(type);
            if (bucket == null) {
                continue;
            }
            if (bucket.matchedCount == 0 && bucket.mismatchCount == 0) {
                continue;
            }

            double oldHazard = getHazard(type);
            double etaMatch = matchLearningRate(step);
            double etaMismatch = mismatchLearningRate(type, step, bucket.mismatchCount);

            double stepMatch = -etaMatch * bucket.matchedDeltaSum;
            double stepMismatch = etaMismatch * bucket.mismatchGradSum;
            double netStep = HazardLearningUtils.clip(
                    stepMatch + stepMismatch,
                    -MAX_ABS_HAZARD_STEP,
                    MAX_ABS_HAZARD_STEP);

            double newHazard = HazardLearningUtils.clip(oldHazard + netStep, HAZARD_MIN, HAZARD_MAX);
            setHazard(type, newHazard);

            logRuntime(String.format(
                    "step=%d mode=aggregated type=%s matched_count=%d mismatch_count=%d matched_sum=%.6f mismatch_sum=%.6f eta_match=%.6f eta_mismatch=%.6f hazard_old=%.6f hazard_new=%.6f",
                    step,
                    type.name(),
                    bucket.matchedCount,
                    bucket.mismatchCount,
                    bucket.matchedDeltaSum,
                    bucket.mismatchGradSum,
                    etaMatch,
                    etaMismatch,
                    oldHazard,
                    newHazard));
        }
    }

    private double matchLearningRate(long step) {
        double eta = LEARNING_RATE_MATCH_BASE / (1.0 + (MATCH_TIME_DECAY * step));
        return Math.max(LEARNING_RATE_MATCH_MIN, eta);
    }

    private double mismatchLearningRate(MemoryObjectType type, long step, int mismatchCount) {
        double base = LEARNING_RATE_MISMATCH_BASE / (1.0 + (MISMATCH_TIME_DECAY * step));
        base = Math.max(LEARNING_RATE_MISMATCH_MIN, base);

        double multiplier = mismatchEventMultipliers.get(type);
        if (mismatchCount >= MISMATCH_BURST_THRESHOLD) {
            multiplier = Math.max(EVENT_MULTIPLIER_MIN, multiplier * EVENT_DECAY_FACTOR);
        } else if (mismatchCount == 0) {
            multiplier = Math.min(1.0, multiplier * EVENT_RECOVERY_FACTOR);
        }
        mismatchEventMultipliers.put(type, multiplier);

        return base * multiplier;
    }

    private boolean shouldReplan(long step) {
        PlanTarget current = getCurrentPlanTarget();
        if (current == null) {
            return true;
        }
        if (current.type == MemoryObjectType.TILE && this.carriedTiles.size() >= 3) {
            return true;
        }
        if (current.type == MemoryObjectType.HOLE && !this.hasTile()) {
            return true;
        }
        return !isPlanTargetStillValid(current, step);
    }

    private boolean isPlanTargetStillValid(PlanTarget target, long step) {
        if (target == null) {
            return false;
        }

        MemorySideCardEntry entry = sideCard.get(target.type, target.x, target.y);
        if (entry == null) {
            return false;
        }
        if (shouldLazyDelete(entry, step)) {
            return false;
        }
        if (isCellLockedByTeammate(target.x, target.y)) {
            return false;
        }
        return true;
    }

    private PlanTarget getCurrentPlanTarget() {
        if (activePlan == null) {
            return null;
        }
        if (activePlanIndex < 0 || activePlanIndex >= activePlan.targets.size()) {
            return null;
        }
        return activePlan.targets.get(activePlanIndex);
    }

    private void clearActivePlan(String reason) {
        if (activePlan != null) {
            logRuntime(String.format(
                    "step=%d mode=plan_clear reason=%s eu=%.6f",
                    currentStep(),
                    reason,
                    activePlan.eu));
        }
        activePlan = null;
        activePlanIndex = 0;
    }

    private void advancePlanAfterAction(MemoryObjectType type, int x, int y) {
        PlanTarget current = getCurrentPlanTarget();
        if (current == null) {
            return;
        }
        if (current.type == type && current.x == x && current.y == y) {
            activePlanIndex += 1;
            if (activePlan != null && activePlanIndex >= activePlan.targets.size()) {
                clearActivePlan("plan_finished");
            }
        }
    }

    private boolean isTargetInActivePlan(MemoryObjectType type, int x, int y) {
        if (activePlan == null) {
            return false;
        }
        for (int i = Math.max(0, activePlanIndex); i < activePlan.targets.size(); i++) {
            PlanTarget t = activePlan.targets.get(i);
            if (t.type == type && t.x == x && t.y == y) {
                return true;
            }
        }
        return false;
    }

    private PlanCandidate buildBestPlan(long step) {
        List<MemorySideCardEntry> tilePool = recallPlanningPool(MemoryObjectType.TILE, PLAN_TILE_RECALL_LIMIT, step);
        List<MemorySideCardEntry> holePool = recallPlanningPool(MemoryObjectType.HOLE, PLAN_HOLE_RECALL_LIMIT, step);
        if (tilePool.isEmpty() && holePool.isEmpty()) {
            return null;
        }

        PlanBestHolder best = new PlanBestHolder();
        List<PlanTarget> sequence = new ArrayList<PlanTarget>();
        Map<String, Double> aliveProbCache = new HashMap<String, Double>();

        enumeratePlans(
                step,
                this.getX(),
                this.getY(),
                this.carriedTiles.size(),
                0,
                0.0,
                sequence,
                tilePool,
                holePool,
                0,
                0,
                aliveProbCache,
                best);

        if (best.best != null) {
            logRuntime(String.format(
                    "step=%d mode=plan_select len=%d eu=%.6f dist=%d",
                    step,
                    best.best.targets.size(),
                    best.best.eu,
                    best.best.totalDistance));
        }
        return best.best;
    }

    private void enumeratePlans(
            long nowStep,
            int posX,
            int posY,
            int carry,
            int totalDistance,
            double euBase,
            List<PlanTarget> sequence,
            List<MemorySideCardEntry> tilePool,
            List<MemorySideCardEntry> holePool,
            int usedTileMask,
            int usedHoleMask,
            Map<String, Double> aliveProbCache,
            PlanBestHolder best) {

        if (!sequence.isEmpty()) {
            double eu = euBase - fuelPenalty(totalDistance);
            PlanCandidate candidate = new PlanCandidate(new ArrayList<PlanTarget>(sequence), eu, totalDistance);
            if (isBetterPlan(candidate, best.best)) {
                best.best = candidate;
            }
        }

        if (sequence.size() >= PLAN_MAX_DEPTH) {
            return;
        }

        if (best.best != null) {
            int stepsLeft = PLAN_MAX_DEPTH - sequence.size();
            int remainingTiles = tilePool.size() - Integer.bitCount(usedTileMask);
            int remainingHoles = holePool.size() - Integer.bitCount(usedHoleMask);
            double optimisticUpper = euBase
                    - fuelPenalty(totalDistance)
                    + optimisticRemainingGain(carry, stepsLeft, remainingTiles, remainingHoles);
            if (optimisticUpper <= best.best.eu) {
                return;
            }
        }

        if (carry < 3) {
            for (int i = 0; i < tilePool.size(); i++) {
                int bit = 1 << i;
                if ((usedTileMask & bit) != 0) {
                    continue;
                }
                MemorySideCardEntry tile = tilePool.get(i);
                int d = manhattan(posX, posY, tile.getX(), tile.getY());
                int arrival = (int) (nowStep + totalDistance + d);
                double pAlive = aliveProbabilityAt(tile, arrival, aliveProbCache);
                double deltaEu = (PLAN_PICKUP_UTILITY * pAlive) - (PLAN_DISTANCE_COST * d);

                sequence.add(new PlanTarget(tile.getType(), tile.getX(), tile.getY()));

                enumeratePlans(
                        nowStep,
                        tile.getX(),
                        tile.getY(),
                        carry + 1,
                        totalDistance + d,
                        euBase + deltaEu,
                        sequence,
                        tilePool,
                        holePool,
                        usedTileMask | bit,
                        usedHoleMask,
                        aliveProbCache,
                        best);

                sequence.remove(sequence.size() - 1);
            }
        }

        if (carry > 0) {
            for (int i = 0; i < holePool.size(); i++) {
                int bit = 1 << i;
                if ((usedHoleMask & bit) != 0) {
                    continue;
                }
                MemorySideCardEntry hole = holePool.get(i);
                int d = manhattan(posX, posY, hole.getX(), hole.getY());
                int arrival = (int) (nowStep + totalDistance + d);
                double pAlive = aliveProbabilityAt(hole, arrival, aliveProbCache);
                double deltaEu = (PLAN_FILL_UTILITY * pAlive) - (PLAN_DISTANCE_COST * d);

                sequence.add(new PlanTarget(hole.getType(), hole.getX(), hole.getY()));

                enumeratePlans(
                        nowStep,
                        hole.getX(),
                        hole.getY(),
                        carry - 1,
                        totalDistance + d,
                        euBase + deltaEu,
                        sequence,
                        tilePool,
                        holePool,
                        usedTileMask,
                        usedHoleMask | bit,
                        aliveProbCache,
                        best);

                sequence.remove(sequence.size() - 1);
            }
        }
    }

    private double aliveProbabilityAt(MemorySideCardEntry entry, int arrival, Map<String, Double> cache) {
        String key = entry.key() + "@" + arrival;
        Double cached = cache.get(key);
        if (cached != null) {
            return cached.doubleValue();
        }
        double p = HazardLearningUtils.computeAliveProbability(
                entry,
                arrival,
                getHazard(entry.getType()),
                EPSILON);
        cache.put(key, p);
        return p;
    }

    private double optimisticRemainingGain(int carry, int stepsLeft, int remainingTiles, int remainingHoles) {
        if (stepsLeft <= 0) {
            return 0.0;
        }
        int maxFills = Math.min(remainingHoles, Math.min(stepsLeft, carry + remainingTiles));
        int leftoverSteps = Math.max(0, stepsLeft - maxFills);
        int maxPickups = Math.min(remainingTiles, leftoverSteps);
        return (maxFills * PLAN_FILL_UTILITY) + (maxPickups * PLAN_PICKUP_UTILITY);
    }

    private List<MemorySideCardEntry> recallPlanningPool(MemoryObjectType type, int limit, long step) {
        List<MemorySideCardEntry> pool = sideCard.listByType(type);
        if (pool.isEmpty()) {
            return new ArrayList<MemorySideCardEntry>();
        }

        List<MemorySideCardEntry> filtered = new ArrayList<MemorySideCardEntry>();
        for (MemorySideCardEntry entry : pool) {
            if (shouldLazyDelete(entry, step)) {
                removeRecordAndSync(entry.getType(), entry.getX(), entry.getY());
                continue;
            }
            if (isCellLockedByTeammate(entry.getX(), entry.getY())) {
                continue;
            }
            filtered.add(entry);
        }

        Collections.sort(filtered, new Comparator<MemorySideCardEntry>() {
            @Override
            public int compare(MemorySideCardEntry a, MemorySideCardEntry b) {
                int da = Math.abs(a.getX() - getX()) + Math.abs(a.getY() - getY());
                int db = Math.abs(b.getX() - getX()) + Math.abs(b.getY() - getY());
                if (da != db) {
                    return Integer.compare(da, db);
                }
                return Double.compare(
                        HazardLearningUtils.computeAliveProbability(b, currentStep(), getHazard(b.getType()), EPSILON),
                        HazardLearningUtils.computeAliveProbability(a, currentStep(), getHazard(a.getType()), EPSILON));
            }
        });

        int realLimit = Math.min(limit, filtered.size());
        return new ArrayList<MemorySideCardEntry>(filtered.subList(0, realLimit));
    }

    private boolean isCellLockedByTeammate(int x, int y) {
        TargetLease teammate = teammateTargetLeases.get(MemorySideCard.cellKey(x, y));
        return teammate != null;
    }

    private boolean isBetterPlan(PlanCandidate current, PlanCandidate best) {
        if (current == null) {
            return false;
        }
        if (best == null) {
            return true;
        }
        int euCmp = Double.compare(current.eu, best.eu);
        if (euCmp != 0) {
            return euCmp > 0;
        }
        int lenCmp = Integer.compare(current.targets.size(), best.targets.size());
        if (lenCmp != 0) {
            return lenCmp > 0;
        }
        return current.totalDistance < best.totalDistance;
    }

    private int manhattan(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    private double fuelPenalty(int travelDistance) {
        double remainingFuel = this.getFuelLevel() - travelDistance;
        if (remainingFuel >= FUEL_MARGIN) {
            return 0.0;
        }
        return (FUEL_MARGIN - remainingFuel) * PLAN_FUEL_PENALTY;
    }

    private boolean shouldLazyDelete(MemorySideCardEntry entry, long step) {
        if (entry.isSpawnObserved() && step > (entry.getFirstSeen() + Parameters.lifeTime)) {
            return true;
        }

        if ((step - entry.getLastSeen()) > Parameters.lifeTime) {
            return true;
        }

        double pAlive = HazardLearningUtils.computeAliveProbability(
                entry,
                step,
                getHazard(entry.getType()),
                EPSILON);
        return pAlive < P_THETA;
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

    private void syncWorkingMemorySet(TWEntity entity) {
        if (entity == null) {
            return;
        }

        if (entity instanceof TWObject) {
            TWEntity[][] slot = new TWEntity[1][1];
            slot[0][0] = entity;
            this.memory.updateMemory(slot, entity.getX(), entity.getY());
        }

        this.memory.getMemoryGrid().set(entity.getX(), entity.getY(), entity);
    }

    private void removeRecordAndSync(MemoryObjectType type, int x, int y) {
        if (isTargetInActivePlan(type, x, y)) {
            releaseOwnTargetLock("plan_invalidated_target_removed");
            clearActivePlan("target_removed_from_plan");
        }
        if (isOwnLockAt(type, x, y)) {
            releaseOwnTargetLock("target_removed");
        }
        sideCard.remove(type, x, y);
        this.memory.removeAgentPercept(x, y);
        this.memory.getMemoryGrid().set(x, y, null);
    }

    private void rememberFuelStationFromMessage(int x, int y) {
        ObjectGrid2D memoryGrid = this.memory.getMemoryGrid();
        if (!(memoryGrid.get(x, y) instanceof TWFuelStation)) {
            memoryGrid.set(x, y, new TWFuelStation(x, y, this.getEnvironment()));
        }
    }

    private boolean wasObservedEmptyAtStep(int x, int y, long step) {
        String cellKey = MemorySideCard.cellKey(x, y);
        Long selfStep = selfEmptyObservedStep.get(cellKey);
        if (selfStep != null && selfStep == step) {
            return true;
        }
        Long teamStep = teammateEmptyObservedStep.get(cellKey);
        return teamStep != null && teamStep == step;
    }

    private boolean isInnerSensorCell(int x, int y) {
        int dx = Math.abs(x - this.getX());
        int dy = Math.abs(y - this.getY());
        int chebyshev = Math.max(dx, dy);
        return chebyshev <= Math.max(0, Parameters.defaultSensorRange - 1);
    }

    private boolean isInSensorRange(int x, int y) {
        int dx = Math.abs(x - this.getX());
        int dy = Math.abs(y - this.getY());
        return Math.max(dx, dy) <= Parameters.defaultSensorRange;
    }

    private void pruneEmptyObservationCaches(long step) {
        pruneCellObservationMap(selfEmptyObservedStep, step);
        pruneCellObservationMap(teammateEmptyObservedStep, step);
    }

    private void pruneCellObservationMap(Map<String, Long> map, long step) {
        List<String> toRemove = new ArrayList<String>();
        for (Map.Entry<String, Long> entry : map.entrySet()) {
            if ((step - entry.getValue()) > 3) {
                toRemove.add(entry.getKey());
            }
        }
        for (String key : toRemove) {
            map.remove(key);
        }
    }

    private double getHazard(MemoryObjectType type) {
        switch (type) {
            case TILE:
                return hazardTile;
            case HOLE:
                return hazardHole;
            case OBSTACLE:
                return hazardObstacle;
            default:
                return 0.0;
        }
    }

    private double sourceWeight(String source, boolean matched) {
        if (source == null) {
            return 1.0;
        }

        // Team messages are useful but should not dominate updates for global hazard.
        if (matched) {
            if (source.startsWith("team_snapshot")) {
                return 0.20;
            }
            if (source.startsWith("team_point_obs")) {
                return 0.35;
            }
            if (source.startsWith("self_")) {
                return 1.0;
            }
            return 0.50;
        }

        // Mismatch evidence is stronger, but still discounted if only from teammates.
        if (source.startsWith("team_empty")) {
            return 0.60;
        }
        if (source.startsWith("self_")) {
            return 1.0;
        }
        return 0.75;
    }

    private void setHazard(MemoryObjectType type, double value) {
        switch (type) {
            case TILE:
                hazardTile = value;
                return;
            case HOLE:
                hazardHole = value;
                return;
            case OBSTACLE:
                hazardObstacle = value;
                return;
            default:
                return;
        }
    }

    private long currentStep() {
        return this.getEnvironment().schedule.getSteps();
    }

    private void logRuntime(String text) {
        if (!ENABLE_RUNTIME_LOG) {
            return;
        }
        runtimeLogger.log(text);
    }

    private static String buildRuntimeLogPath(String agentName) {
        String safeName = sanitizeFileName(agentName);
        return LOG_DIR + "/" + safeName + "-model-updates.log";
    }

    private static String sanitizeFileName(String raw) {
        if (raw == null || raw.isEmpty()) {
            return "agent";
        }

        // Keep spaces, but replace invalid filename characters on Windows.
        String sanitized = raw.replaceAll("[\\\\/:*?\"<>|]", "_").trim();
        if (sanitized.isEmpty()) {
            return "agent";
        }
        return sanitized;
    }

    private static class PlanTarget {
        private final MemoryObjectType type;
        private final int x;
        private final int y;

        private PlanTarget(MemoryObjectType type, int x, int y) {
            this.type = type;
            this.x = x;
            this.y = y;
        }
    }

    private static class PlanCandidate {
        private final List<PlanTarget> targets;
        private final double eu;
        private final int totalDistance;

        private PlanCandidate(List<PlanTarget> targets, double eu, int totalDistance) {
            this.targets = targets;
            this.eu = eu;
            this.totalDistance = totalDistance;
        }
    }

    private static class PlanBestHolder {
        private PlanCandidate best;
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

    private static class TeammatePosition {
        private final int x;
        private final int y;
        private final long step;

        private TeammatePosition(int x, int y, long step) {
            this.x = x;
            this.y = y;
            this.step = step;
        }
    }

    private static class SectorRect {
        private final int id;
        private final int x1;
        private final int y1;
        private final int x2;
        private final int y2;

        private SectorRect(int id, int x1, int y1, int x2, int y2) {
            this.id = id;
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;
        }

        private boolean contains(int x, int y) {
            return x >= x1 && x <= x2 && y >= y1 && y <= y2;
        }

        private int area() {
            int w = Math.max(0, x2 - x1 + 1);
            int h = Math.max(0, y2 - y1 + 1);
            return w * h;
        }

        private Int2D center() {
            int cx = (x1 + x2) / 2;
            int cy = (y1 + y2) / 2;
            return new Int2D(cx, cy);
        }
    }

    private static class TypeGradientBucket {
        private double matchedDeltaSum;
        private double mismatchGradSum;
        private int matchedCount;
        private int mismatchCount;

        private void reset() {
            matchedDeltaSum = 0.0;
            mismatchGradSum = 0.0;
            matchedCount = 0;
            mismatchCount = 0;
        }
    }
}
