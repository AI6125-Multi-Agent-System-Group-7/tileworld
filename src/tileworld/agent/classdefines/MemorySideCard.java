package tileworld.agent.classdefines;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Sidecar store that augments TWAgentWorkingMemory with hazard-model metadata.
 */
public class MemorySideCard {
    private final Map<String, MemorySideCardEntry> entries;

    public MemorySideCard() {
        this.entries = new HashMap<String, MemorySideCardEntry>();
    }

    public MemorySideCardEntry get(MemoryObjectType type, int x, int y) {
        return entries.get(key(type, x, y));
    }

    public boolean contains(MemoryObjectType type, int x, int y) {
        return entries.containsKey(key(type, x, y));
    }

    public MemorySideCardEntry upsert(MemoryObjectType type, int x, int y, double observedAt, boolean spawnObserved) {
        String key = key(type, x, y);
        MemorySideCardEntry existing = entries.get(key);
        if (existing == null) {
            MemorySideCardEntry created = new MemorySideCardEntry(type, x, y, observedAt, observedAt, spawnObserved);
            entries.put(key, created);
            return created;
        }

        existing.touch(observedAt);
        if (spawnObserved) {
            existing.setSpawnObserved(true);
        }
        return existing;
    }

    public void remove(MemoryObjectType type, int x, int y) {
        entries.remove(key(type, x, y));
    }

    public List<MemorySideCardEntry> listByType(MemoryObjectType type) {
        List<MemorySideCardEntry> out = new ArrayList<MemorySideCardEntry>();
        for (MemorySideCardEntry entry : entries.values()) {
            if (entry.getType() == type) {
                out.add(entry);
            }
        }
        return out;
    }

    public List<MemorySideCardEntry> listAll() {
        return new ArrayList<MemorySideCardEntry>(entries.values());
    }

    public static String key(MemoryObjectType type, int x, int y) {
        return type.name() + "#" + x + "#" + y;
    }

    public static String cellKey(int x, int y) {
        return x + "#" + y;
    }
}
