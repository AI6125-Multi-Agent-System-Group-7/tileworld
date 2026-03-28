package tileworld.agent;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

import sim.util.Bag;
import sim.field.grid.ObjectGrid2D;
import sim.util.IntBag;
import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.agent.utils.SensorSnapshotCodec;

/**
 * Group7AgentBase
 *
 * Let's put shared utilities for all Group 7 agents in this class!
 * Keep your policy small; keep your robots alive.
 */
public abstract class Group7AgentBase extends TWAgent {
    private static final String PROTOCOL_VERSION = "G7P2";
    private static final String PROTOCOL_SEPARATOR = "|";
    private static final String TO_ALL = "ALL";
    private static final int ZONE_ASSIGN_MAX_RETRY = 3;

    /**
     * Compact message type codes for efficient serialization/parsing.
     */
    public enum CommType {
        // Observed a previously unseen object.
        OBS_NEW_TILE("NT"),
        OBS_NEW_HOLE("NH"),
        OBS_OBSTACLE("OB"),
        // Fuel station should only be broadcast once by each agent.
        OBS_FUEL_ONCE("FS"),
        // Per-step sensor snapshot broadcast by base class.
        OBS_SENSOR_SNAPSHOT("SS"),
        // Target lock lifecycle events for cooperative deconfliction.
        TARGET_LOCK("TL"),
        TARGET_RELEASE("TR"),
        // Bootstrap zone assignment and acknowledgement.
        ZONE_ASSIGN("ZA"),
        ZONE_ACK("ZK"),
        // Action-level events from this agent.
        ACTION_PICKUP_TILE("PK"),
        ACTION_FILL_HOLE("FH");

        private final String code;

        CommType(String code) {
            this.code = code;
        }

        public String code() {
            return code;
        }

        public static CommType fromCode(String code) {
            for (CommType t : values()) {
                if (t.code.equals(code)) {
                    return t;
                }
            }
            return null;
        }
    }

    /**
     * Parsed protocol message for downstream agents to consume.
     */
    protected static class ParsedMessage {
        public final String from;
        public final String to;
        public final long step;
        public final CommType type;
        public final int x;
        public final int y;
        public final String payload;

        ParsedMessage(String from, String to, long step, CommType type, int x, int y, String payload) {
            this.from = from;
            this.to = to;
            this.step = step;
            this.type = type;
            this.x = x;
            this.y = y;
            this.payload = payload;
        }
    }

    private final String name;

    protected List<TWDirection> currentPath;
    protected Int2D currentTarget;

    // Communication state.
    private final Set<String> announcedObjectKeys;
    private boolean fuelStationBroadcasted;

    // Optional bootstrap zone coordination state.
    private boolean zoneCoordinationEnabled;
    private boolean zoneCoordinationCompleted;
    private boolean zonePlanBuilt;
    private boolean zoneAckSent;
    private long zoneEpoch;
    private String zoneManagerName;
    private ZoneAssignment myZoneAssignment;
    private String zoneOrientation;
    private final Map<String, Int2D> zoneKnownAgentPositions;
    private final Map<String, ZoneAssignment> zoneAssignmentsByAgent;
    private final Set<String> zoneAckedAgents;
    private final Map<String, Integer> zoneSendAttempts;

    // Zig-zag exploration state.
    private final int zigZagStep;
    private int zigZagX;
    private int zigZagY;
    private boolean zigZagRight;
    private boolean zigZagDown;
    private Int2D zigZagTarget;

    public Group7AgentBase(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
        this.currentPath = null;
        this.currentTarget = null;

        this.announcedObjectKeys = new HashSet<String>();
        this.fuelStationBroadcasted = false;
        this.zoneCoordinationEnabled = false;
        this.zoneCoordinationCompleted = false;
        this.zonePlanBuilt = false;
        this.zoneAckSent = false;
        this.zoneEpoch = 0L;
        this.zoneManagerName = null;
        this.myZoneAssignment = null;
        this.zoneOrientation = "X3Y2";
        this.zoneKnownAgentPositions = new HashMap<String, Int2D>();
        this.zoneAssignmentsByAgent = new HashMap<String, ZoneAssignment>();
        this.zoneAckedAgents = new HashSet<String>();
        this.zoneSendAttempts = new HashMap<String, Integer>();

        this.zigZagStep = Math.max(1, (Parameters.defaultSensorRange * 2) + 1);
        this.zigZagX = Math.max(0, Math.min(env.getxDimension() - 1, xpos));
        this.zigZagY = Math.max(0, Math.min(env.getyDimension() - 1, ypos));
        this.zigZagRight = true;
        this.zigZagDown = true;
        this.zigZagTarget = null;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public final void communicate() {
        rememberFuelStationsInSensorRange();

        List<Message> outbox = new ArrayList<Message>(12);
        collectObservationMessages(outbox);
        collectSensorSnapshotMessage(outbox);
        appendZoneCoordinationMessages(outbox);
        appendCustomMessages(outbox);

        for (Message message : outbox) {
            this.getEnvironment().receiveMessage(message);
        }
    }

    /**
     * Optional extension hook for subclass-specific outbound messages.
     * Base-level communication is always emitted by {@link #communicate()}.
     */
    protected void appendCustomMessages(List<Message> outbox) {
        // Default: no custom messages.
    }

    /**
     * Parses a protocol message generated by Group7 agents.
     *
     * Subclasses can use this helper in think() to process teammate messages.
     * Returns null when the message is malformed or not produced by this protocol.
     */
    protected ParsedMessage parseProtocolMessage(Message message) {
        if (message == null) {
            return null;
        }

        String raw = message.getMessage();
        if (raw == null || raw.isEmpty()) {
            return null;
        }

        String[] parts = raw.split("\\|", 8);
        if (parts.length < 7) {
            return null;
        }
        if (!PROTOCOL_VERSION.equals(parts[0])) {
            return null;
        }

        long step;
        int x;
        int y;
        try {
            step = Long.parseLong(parts[1]);
            x = Integer.parseInt(parts[5]);
            y = Integer.parseInt(parts[6]);
        } catch (NumberFormatException ex) {
            return null;
        }

        CommType type = CommType.fromCode(parts[4]);
        if (type == null) {
            return null;
        }

        String payload = (parts.length == 8) ? parts[7] : "";
        return new ParsedMessage(parts[2], parts[3], step, type, x, y, payload);
    }

    /**
     * Zone assignment produced by bootstrap manager.
     */
    protected static class ZoneAssignment {
        public final long epoch;
        public final String manager;
        public final String orientation;
        public final int x1;
        public final int y1;
        public final int x2;
        public final int y2;

        ZoneAssignment(long epoch, String manager, String orientation, int x1, int y1, int x2, int y2) {
            this.epoch = epoch;
            this.manager = manager;
            this.orientation = orientation;
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;
        }

        public boolean contains(int x, int y) {
            return x >= x1 && x <= x2 && y >= y1 && y <= y2;
        }

        public Int2D center() {
            int cx = (x1 + x2) / 2;
            int cy = (y1 + y2) / 2;
            return new Int2D(cx, cy);
        }
    }

    /**
     * Enables bootstrap zone assignment coordination (ZA/ZK).
     * Designed to be opt-in so existing subclasses keep old behavior.
     */
    protected final void enableZoneCoordination() {
        this.zoneCoordinationEnabled = true;
    }

    protected final boolean isZoneCoordinationEnabled() {
        return zoneCoordinationEnabled;
    }

    protected final boolean isZoneCoordinationComplete() {
        return zoneCoordinationCompleted;
    }

    protected final void markZoneCoordinationComplete() {
        zoneCoordinationCompleted = true;
    }

    protected final boolean hasZoneAssignment() {
        return myZoneAssignment != null;
    }

    protected final ZoneAssignment getZoneAssignment() {
        return myZoneAssignment;
    }

    protected final boolean isZoneManager() {
        return zoneManagerName != null && zoneManagerName.equals(getName());
    }

    protected final String getZoneManagerName() {
        return zoneManagerName;
    }

    protected final void processZoneCoordinationInThink() {
        if (!zoneCoordinationEnabled || zoneCoordinationCompleted) {
            return;
        }

        zoneKnownAgentPositions.put(getName(), new Int2D(this.getX(), this.getY()));

        List<Message> inbox = new ArrayList<Message>(this.getEnvironment().getMessages());
        for (Message message : inbox) {
            ParsedMessage parsed = parseProtocolMessage(message);
            if (parsed == null || parsed.from == null) {
                continue;
            }

            if (parsed.type == CommType.OBS_SENSOR_SNAPSHOT) {
                zoneKnownAgentPositions.put(parsed.from, new Int2D(parsed.x, parsed.y));
                continue;
            }

            if (parsed.type == CommType.ZONE_ASSIGN && isMessageForMe(parsed)) {
                ZoneAssignment assignment = parseZoneAssignmentPayload(parsed);
                if (assignment != null) {
                    myZoneAssignment = assignment;
                    zoneManagerName = assignment.manager;
                    zoneEpoch = assignment.epoch;
                    zoneOrientation = assignment.orientation;
                    zoneAckSent = false;
                    sendZoneAckIfNeeded();
                }
                continue;
            }

            if (parsed.type == CommType.ZONE_ACK && isZoneManager() && isMessageForMe(parsed)) {
                Long ackEpoch = parseEpochFromAckPayload(parsed.payload);
                if (ackEpoch != null && ackEpoch == zoneEpoch) {
                    zoneAckedAgents.add(parsed.from);
                }
            }
        }

        if (zoneManagerName == null && !zoneKnownAgentPositions.isEmpty()) {
            zoneManagerName = chooseLexicographicallySmallest(zoneKnownAgentPositions.keySet());
        }

        if (isZoneManager() && !zonePlanBuilt && zoneKnownAgentPositions.size() >= 1) {
            zoneEpoch = zoneEpoch + 1L;
            buildZoneAssignmentPlan();
            zonePlanBuilt = true;
            zoneAckedAgents.add(getName());
            ZoneAssignment mine = zoneAssignmentsByAgent.get(getName());
            if (mine != null) {
                myZoneAssignment = mine;
            }
        }
    }

    private void sendZoneAckIfNeeded() {
        if (zoneAckSent || myZoneAssignment == null || myZoneAssignment.manager == null) {
            return;
        }
        if (myZoneAssignment.manager.equals(getName())) {
            zoneAckSent = true;
            return;
        }
        publishProtocolMessageTo(
                myZoneAssignment.manager,
                CommType.ZONE_ACK,
                this.getX(),
                this.getY(),
                "e=" + myZoneAssignment.epoch + ",ok=1");
        zoneAckSent = true;
    }

    private void appendZoneCoordinationMessages(List<Message> outbox) {
        if (!zoneCoordinationEnabled || zoneCoordinationCompleted || !isZoneManager() || !zonePlanBuilt) {
            return;
        }

        for (Map.Entry<String, ZoneAssignment> entry : zoneAssignmentsByAgent.entrySet()) {
            String agent = entry.getKey();
            if (agent == null || agent.equals(getName())) {
                continue;
            }
            if (zoneAckedAgents.contains(agent)) {
                continue;
            }

            int attempts = zoneSendAttempts.containsKey(agent) ? zoneSendAttempts.get(agent) : 0;
            if (attempts >= ZONE_ASSIGN_MAX_RETRY) {
                continue;
            }

            ZoneAssignment zone = entry.getValue();
            outbox.add(createProtocolMessageTo(
                    agent,
                    CommType.ZONE_ASSIGN,
                    zone.center().x,
                    zone.center().y,
                    encodeZoneAssignmentPayload(zone)));
            zoneSendAttempts.put(agent, attempts + 1);
        }
    }

    private String chooseLexicographicallySmallest(Set<String> names) {
        List<String> sorted = new ArrayList<String>(names);
        Collections.sort(sorted);
        return sorted.isEmpty() ? getName() : sorted.get(0);
    }

    private void buildZoneAssignmentPlan() {
        zoneAssignmentsByAgent.clear();
        zoneAckedAgents.clear();
        zoneSendAttempts.clear();

        int xDim = this.getEnvironment().getxDimension();
        int yDim = this.getEnvironment().getyDimension();
        int agentsCount = Math.max(1, zoneKnownAgentPositions.size());

        int splitX;
        int splitY;
        if (agentsCount < 6) {
            if (xDim >= yDim) {
                splitX = agentsCount;
                splitY = 1;
            } else {
                splitX = 1;
                splitY = agentsCount;
            }
            zoneOrientation = "X" + splitX + "Y" + splitY;
        } else {
            if (xDim > yDim) {
                splitX = 3;
                splitY = 2;
                zoneOrientation = "X3Y2";
            } else if (yDim > xDim) {
                splitX = 2;
                splitY = 3;
                zoneOrientation = "X2Y3";
            } else {
                // Square map: deterministic pseudo-random split decided by manager.
                boolean xLong = ((zoneEpoch + Math.abs(getName().hashCode())) % 2) == 0;
                splitX = xLong ? 3 : 2;
                splitY = xLong ? 2 : 3;
                zoneOrientation = xLong ? "X3Y2" : "X2Y3";
            }
        }

        List<ZoneAssignment> zones = generateZoneGrid(splitX, splitY);
        Set<Integer> usedZoneIndices = new HashSet<Integer>();

        List<String> agents = new ArrayList<String>(zoneKnownAgentPositions.keySet());
        Collections.sort(agents);

        for (String agent : agents) {
            Int2D pos = zoneKnownAgentPositions.get(agent);
            if (pos == null) {
                continue;
            }
            int chosen = chooseNearestAvailableZoneIndex(pos, zones, usedZoneIndices);
            if (chosen < 0) {
                continue;
            }
            usedZoneIndices.add(chosen);
            zoneAssignmentsByAgent.put(agent, zones.get(chosen));
        }
    }

    private int chooseNearestAvailableZoneIndex(Int2D pos, List<ZoneAssignment> zones, Set<Integer> used) {
        int bestIndex = -1;
        int bestDist = Integer.MAX_VALUE;
        int bestCx = Integer.MAX_VALUE;
        int bestCy = Integer.MAX_VALUE;

        for (int i = 0; i < zones.size(); i++) {
            if (used.contains(i)) {
                continue;
            }
            ZoneAssignment z = zones.get(i);
            Int2D c = z.center();
            int d = Math.abs(pos.x - c.x) + Math.abs(pos.y - c.y);
            if (d < bestDist) {
                bestDist = d;
                bestIndex = i;
                bestCx = c.x;
                bestCy = c.y;
                continue;
            }
            if (d == bestDist) {
                if (c.x < bestCx || (c.x == bestCx && c.y < bestCy)) {
                    bestIndex = i;
                    bestCx = c.x;
                    bestCy = c.y;
                }
            }
        }
        return bestIndex;
    }

    private List<ZoneAssignment> generateZoneGrid(int splitX, int splitY) {
        List<ZoneAssignment> zones = new ArrayList<ZoneAssignment>(splitX * splitY);
        int[] xSizes = splitSizes(this.getEnvironment().getxDimension(), splitX);
        int[] ySizes = splitSizes(this.getEnvironment().getyDimension(), splitY);

        int xStart = 0;
        for (int ix = 0; ix < splitX; ix++) {
            int xEnd = xStart + xSizes[ix] - 1;
            int yStart = 0;
            for (int iy = 0; iy < splitY; iy++) {
                int yEnd = yStart + ySizes[iy] - 1;
                zones.add(new ZoneAssignment(zoneEpoch, getName(), zoneOrientation, xStart, yStart, xEnd, yEnd));
                yStart = yEnd + 1;
            }
            xStart = xEnd + 1;
        }
        return zones;
    }

    private int[] splitSizes(int size, int parts) {
        int[] out = new int[parts];
        int base = size / parts;
        int remain = size % parts;
        for (int i = 0; i < parts; i++) {
            out[i] = base + ((i < remain) ? 1 : 0);
        }
        return out;
    }

    private String encodeZoneAssignmentPayload(ZoneAssignment z) {
        StringBuilder sb = new StringBuilder(96);
        sb.append("e=").append(z.epoch)
          .append(",x1=").append(z.x1)
          .append(",y1=").append(z.y1)
          .append(",x2=").append(z.x2)
          .append(",y2=").append(z.y2)
          .append(",o=").append(z.orientation);
        return sb.toString();
    }

    private ZoneAssignment parseZoneAssignmentPayload(ParsedMessage parsed) {
        Map<String, String> kv = parseKeyValuePayload(parsed.payload);
        Long epoch = parseLong(kv.get("e"));
        Integer x1 = parseInt(kv.get("x1"));
        Integer y1 = parseInt(kv.get("y1"));
        Integer x2 = parseInt(kv.get("x2"));
        Integer y2 = parseInt(kv.get("y2"));
        String o = kv.get("o");

        if (epoch == null || x1 == null || y1 == null || x2 == null || y2 == null) {
            return null;
        }

        String orientation = (o == null || o.isEmpty()) ? zoneOrientation : o;
        return new ZoneAssignment(epoch.longValue(), parsed.from, orientation, x1, y1, x2, y2);
    }

    private Long parseEpochFromAckPayload(String payload) {
        Map<String, String> kv = parseKeyValuePayload(payload);
        return parseLong(kv.get("e"));
    }

    private Map<String, String> parseKeyValuePayload(String payload) {
        Map<String, String> out = new HashMap<String, String>();
        if (payload == null || payload.isEmpty()) {
            return out;
        }

        String[] tokens = payload.split(",");
        for (String token : tokens) {
            int idx = token.indexOf('=');
            if (idx <= 0 || idx >= token.length() - 1) {
                continue;
            }
            String key = token.substring(0, idx).trim();
            String val = token.substring(idx + 1).trim();
            if (!key.isEmpty()) {
                out.put(key, val);
            }
        }
        return out;
    }

    private Integer parseInt(String raw) {
        if (raw == null || raw.isEmpty()) {
            return null;
        }
        try {
            return Integer.parseInt(raw);
        } catch (NumberFormatException ex) {
            return null;
        }
    }

    private Long parseLong(String raw) {
        if (raw == null || raw.isEmpty()) {
            return null;
        }
        try {
            return Long.parseLong(raw);
        } catch (NumberFormatException ex) {
            return null;
        }
    }

    protected final boolean isMessageForMe(ParsedMessage parsed) {
        if (parsed == null || parsed.to == null) {
            return false;
        }
        return TO_ALL.equals(parsed.to) || getName().equals(parsed.to);
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

    /**
     * Zig-zag exploration with strip width aligned to sensor coverage.
     * This is deterministic and low-overhead, useful as a safe fallback policy.
     */
    protected TWThought exploreZigZag() {
        for (int attempts = 0; attempts < 4; attempts++) {
            if (zigZagTarget == null || at(zigZagTarget)) {
                zigZagTarget = nextZigZagTarget();
            }

            TWThought thought = stepToward(zigZagTarget);
            if (!isWait(thought)) {
                return thought;
            }

            // Current target is likely blocked/unreachable; move to next strip anchor.
            zigZagTarget = nextZigZagTarget();
            clearPlan();
        }

        return waitThought();
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
                        int beforeTiles = this.carriedTiles.size();
                        int tx = obj.getX();
                        int ty = obj.getY();
                        this.pickUpTile((TWTile) obj);
                        if (this.carriedTiles.size() == beforeTiles + 1) {
                            publishActionEvent(CommType.ACTION_PICKUP_TILE, tx, ty, "");
                        }
                    }
                    break;
                case PUTDOWN:
                    TWEntity hole = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                    if (hole instanceof TWHole && this.hasTile()) {
                        int beforeScore = this.score;
                        int hx = hole.getX();
                        int hy = hole.getY();
                        this.putTileInHole((TWHole) hole);
                        if (this.score == beforeScore + 1) {
                            publishActionEvent(CommType.ACTION_FILL_HOLE, hx, hy, "");
                        }
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

    private void collectObservationMessages(List<Message> outbox) {
        Bag sensedObjects = new Bag();
        IntBag objectX = new IntBag();
        IntBag objectY = new IntBag();

        this.getEnvironment().getObjectGrid().getNeighborsMaxDistance(
                this.getX(),
                this.getY(),
                Parameters.defaultSensorRange,
                false,
                sensedObjects,
                objectX,
                objectY);

        for (int i = 0; i < sensedObjects.size(); i++) {
            Object obj = sensedObjects.get(i);
            if (!(obj instanceof TWEntity)) {
                continue;
            }
            TWEntity entity = (TWEntity) obj;

            if (entity instanceof TWFuelStation) {
                if (!fuelStationBroadcasted) {
                    outbox.add(encodeProtocolMessage(CommType.OBS_FUEL_ONCE, entity.getX(), entity.getY(), ""));
                    fuelStationBroadcasted = true;
                }
                continue;
            }

            CommType type = mapObservationType(entity);
            if (type == null) {
                continue;
            }

            String key = buildObjectKey(entity.getClass().getSimpleName(), entity.getX(), entity.getY());
            if (!announcedObjectKeys.contains(key)) {
                outbox.add(encodeProtocolMessage(type, entity.getX(), entity.getY(), ""));
                announcedObjectKeys.add(key);
            }
        }
    }

    private CommType mapObservationType(TWEntity entity) {
        if (entity instanceof TWTile) {
            return CommType.OBS_NEW_TILE;
        }
        if (entity instanceof TWHole) {
            return CommType.OBS_NEW_HOLE;
        }
        if (entity instanceof tileworld.environment.TWObstacle) {
            return CommType.OBS_OBSTACLE;
        }
        return null;
    }

    private void collectSensorSnapshotMessage(List<Message> outbox) {
        List<SensorSnapshotCodec.SnapshotItem> items = new ArrayList<SensorSnapshotCodec.SnapshotItem>();
        ObjectGrid2D grid = this.getEnvironment().getObjectGrid();
        int range = Parameters.defaultSensorRange;

        for (int dx = -range; dx <= range; dx++) {
            for (int dy = -range; dy <= range; dy++) {
                int x = this.getX() + dx;
                int y = this.getY() + dy;
                if (!this.getEnvironment().isInBounds(x, y)) {
                    continue;
                }

                Object obj = grid.get(x, y);
                if (obj == null) {
                    items.add(new SensorSnapshotCodec.SnapshotItem("E", x, y));
                    continue;
                }

                if (obj instanceof TWTile) {
                    items.add(new SensorSnapshotCodec.SnapshotItem("T", x, y));
                    continue;
                }
                if (obj instanceof TWHole) {
                    items.add(new SensorSnapshotCodec.SnapshotItem("H", x, y));
                    continue;
                }
                if (obj instanceof TWObstacle) {
                    items.add(new SensorSnapshotCodec.SnapshotItem("O", x, y));
                }
            }
        }

        outbox.add(encodeProtocolMessage(
                CommType.OBS_SENSOR_SNAPSHOT,
                this.getX(),
                this.getY(),
                SensorSnapshotCodec.encode(items)));
    }

    private void publishActionEvent(CommType type, int x, int y, String payload) {
        publishProtocolMessage(type, x, y, payload);
    }

    protected final void publishProtocolMessage(CommType type, int x, int y, String payload) {
        publishProtocolMessageTo(TO_ALL, type, x, y, payload);
    }

    protected final void publishProtocolMessageTo(String to, CommType type, int x, int y, String payload) {
        this.getEnvironment().receiveMessage(encodeProtocolMessageTo(to, type, x, y, payload));
    }

    private Message encodeProtocolMessage(CommType type, int x, int y, String payload) {
        return encodeProtocolMessageTo(TO_ALL, type, x, y, payload);
    }

    private Message encodeProtocolMessageTo(String to, CommType type, int x, int y, String payload) {
        long step = this.getEnvironment().schedule.getSteps();
        String safePayload = sanitizePayload(payload);
        String safeTo = normalizeRecipient(to);

        StringBuilder sb = new StringBuilder(64);
        sb.append(PROTOCOL_VERSION).append(PROTOCOL_SEPARATOR)
          .append(step).append(PROTOCOL_SEPARATOR)
          .append(getName()).append(PROTOCOL_SEPARATOR)
          .append(safeTo).append(PROTOCOL_SEPARATOR)
          .append(type.code()).append(PROTOCOL_SEPARATOR)
          .append(x).append(PROTOCOL_SEPARATOR)
          .append(y);
        if (!safePayload.isEmpty()) {
            sb.append(PROTOCOL_SEPARATOR).append(safePayload);
        }

        return new Message(getName(), safeTo, sb.toString());
    }

    /**
     * Creates a protocol message that follows the base communication format.
     * Intended for subclasses that want to append extra messages in communicate().
     */
    protected final Message createProtocolMessage(CommType type, int x, int y, String payload) {
        return encodeProtocolMessage(type, x, y, payload);
    }

    protected final Message createProtocolMessageTo(String to, CommType type, int x, int y, String payload) {
        return encodeProtocolMessageTo(to, type, x, y, payload);
    }

    protected final String encodeTargetLockPayload(double priority) {
        double safe = Math.max(0.0, Math.min(1.0, priority));
        return String.format(Locale.US, "p=%.6f", safe);
    }

    protected final String encodeTargetLockPayload(double priority, int ttlSteps) {
        double safe = Math.max(0.0, Math.min(1.0, priority));
        int safeTtl = Math.max(1, ttlSteps);
        return String.format(Locale.US, "p=%.6f,ttl=%d", safe, safeTtl);
    }

    protected final Double parseTargetLockPriority(String payload) {
        if (payload == null || payload.isEmpty()) {
            return null;
        }

        String[] tokens = payload.split(",");
        for (String token : tokens) {
            String trimmed = token.trim();
            if (trimmed.startsWith("p=")) {
                try {
                    return Double.parseDouble(trimmed.substring(2));
                } catch (NumberFormatException ignored) {
                    return null;
                }
            }
        }

        // Backward-compatible fallback when payload is a plain number.
        try {
            return Double.parseDouble(payload.trim());
        } catch (NumberFormatException ignored) {
            return null;
        }
    }

    protected final Integer parseTargetLockTtl(String payload) {
        if (payload == null || payload.isEmpty()) {
            return null;
        }

        String[] tokens = payload.split(",");
        for (String token : tokens) {
            String trimmed = token.trim();
            if (trimmed.startsWith("ttl=")) {
                try {
                    int ttl = Integer.parseInt(trimmed.substring(4));
                    return (ttl > 0) ? ttl : null;
                } catch (NumberFormatException ignored) {
                    return null;
                }
            }
        }
        return null;
    }

    private String sanitizePayload(String payload) {
        if (payload == null || payload.isEmpty()) {
            return "";
        }
        return payload.replace(PROTOCOL_SEPARATOR, "/");
    }

    private String normalizeRecipient(String to) {
        if (to == null || to.trim().isEmpty()) {
            return TO_ALL;
        }
        return to.trim();
    }

    private String buildObjectKey(String type, int x, int y) {
        return type + "#" + x + "#" + y;
    }

    private boolean at(Int2D pos) {
        return pos != null && this.getX() == pos.x && this.getY() == pos.y;
    }

    private boolean isWait(TWThought thought) {
        return thought != null && thought.getAction() == TWAction.MOVE && thought.getDirection() == TWDirection.Z;
    }

    private Int2D nextZigZagTarget() {
        int xDim = this.getEnvironment().getxDimension();
        int yDim = this.getEnvironment().getyDimension();

        int candidateX = zigZagX;
        int candidateY = zigZagY;

        if (zigZagRight) {
            int nextX = candidateX + zigZagStep;
            if (nextX < xDim) {
                candidateX = nextX;
            } else {
                candidateX = xDim - 1;
                candidateY = nextZigZagRow(candidateY, yDim);
                zigZagRight = false;
            }
        } else {
            int nextX = candidateX - zigZagStep;
            if (nextX >= 0) {
                candidateX = nextX;
            } else {
                candidateX = 0;
                candidateY = nextZigZagRow(candidateY, yDim);
                zigZagRight = true;
            }
        }

        zigZagX = candidateX;
        zigZagY = candidateY;
        return new Int2D(candidateX, candidateY);
    }

    /**
     * Advances one strip in the current vertical direction; when hitting top/bottom,
     * flips direction and continues, so coverage oscillates across the full map.
     */
    private int nextZigZagRow(int currentY, int yDim) {
        int signedStep = zigZagDown ? zigZagStep : -zigZagStep;
        int nextY = currentY + signedStep;

        if (nextY >= 0 && nextY < yDim) {
            return nextY;
        }

        // Bounce at boundary and try one strip in the opposite direction.
        zigZagDown = !zigZagDown;
        signedStep = zigZagDown ? zigZagStep : -zigZagStep;
        nextY = currentY + signedStep;

        if (nextY < 0) {
            return 0;
        }
        if (nextY >= yDim) {
            return yDim - 1;
        }
        return nextY;
    }

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
