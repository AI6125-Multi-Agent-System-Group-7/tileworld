package tileworld.agent.classdefines;

/**
 * Additional per-object memory tracked outside TWAgentWorkingMemory.
 *
 * This record stores metadata needed by the hazard model and candidate pruning,
 * while TWAgentWorkingMemory remains the base memory for existing planners.
 */
public class MemorySideCardEntry {
    private final MemoryObjectType type;
    private final int x;
    private final int y;

    private double firstSeen;
    private double lastSeen;
    private Double disappearAt;
    private boolean spawnObserved;
    private long lastHazardUpdateStep;

    public MemorySideCardEntry(MemoryObjectType type, int x, int y, double firstSeen, double lastSeen, boolean spawnObserved) {
        this.type = type;
        this.x = x;
        this.y = y;
        this.firstSeen = firstSeen;
        this.lastSeen = lastSeen;
        this.spawnObserved = spawnObserved;
        this.disappearAt = null;
        this.lastHazardUpdateStep = Long.MIN_VALUE / 4;
    }

    public MemoryObjectType getType() {
        return type;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public double getFirstSeen() {
        return firstSeen;
    }

    public double getLastSeen() {
        return lastSeen;
    }

    public void setLastSeen(double lastSeen) {
        this.lastSeen = lastSeen;
    }

    public Double getDisappearAt() {
        return disappearAt;
    }

    public void setDisappearAt(Double disappearAt) {
        this.disappearAt = disappearAt;
    }

    public boolean isSpawnObserved() {
        return spawnObserved;
    }

    public void setSpawnObserved(boolean spawnObserved) {
        this.spawnObserved = spawnObserved;
    }

    public long getLastHazardUpdateStep() {
        return lastHazardUpdateStep;
    }

    public void setLastHazardUpdateStep(long lastHazardUpdateStep) {
        this.lastHazardUpdateStep = lastHazardUpdateStep;
    }

    public void touch(double seenTime) {
        this.lastSeen = seenTime;
        this.disappearAt = null;
    }

    public String key() {
        return type.name() + "#" + x + "#" + y;
    }
}
