package tileworld.agent.utils;

import java.util.ArrayList;
import java.util.List;

/**
 * Encodes/decodes compact sensor snapshots used by AgentHanny teammates.
 *
 * Token format:
 * - Object token: "<TYPE>,<x>,<y>", where TYPE is T/H/O
 * - Empty token: "E,<x>,<y>"
 * Tokens are separated by ';'
 */
public final class SensorSnapshotCodec {
    private static final String TOKEN_SEPARATOR = ";";
    private static final String FIELD_SEPARATOR = ",";

    private SensorSnapshotCodec() {
    }

    public static String encode(List<SnapshotItem> items) {
        StringBuilder sb = new StringBuilder(items.size() * 8);
        for (int i = 0; i < items.size(); i++) {
            SnapshotItem item = items.get(i);
            if (i > 0) {
                sb.append(TOKEN_SEPARATOR);
            }
            sb.append(item.code).append(FIELD_SEPARATOR).append(item.x).append(FIELD_SEPARATOR).append(item.y);
        }
        return sb.toString();
    }

    public static List<SnapshotItem> decode(String payload) {
        List<SnapshotItem> out = new ArrayList<SnapshotItem>();
        if (payload == null || payload.isEmpty()) {
            return out;
        }

        String[] tokens = payload.split(TOKEN_SEPARATOR);
        for (String token : tokens) {
            String[] fields = token.split(FIELD_SEPARATOR);
            if (fields.length != 3) {
                continue;
            }
            try {
                int x = Integer.parseInt(fields[1]);
                int y = Integer.parseInt(fields[2]);
                out.add(new SnapshotItem(fields[0], x, y));
            } catch (NumberFormatException ignored) {
                // Ignore malformed token.
            }
        }
        return out;
    }

    public static final class SnapshotItem {
        public final String code;
        public final int x;
        public final int y;

        public SnapshotItem(String code, int x, int y) {
            this.code = code;
            this.x = x;
            this.y = y;
        }
    }
}
