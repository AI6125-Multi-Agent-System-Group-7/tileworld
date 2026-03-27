package tileworld.agent.classdefines;

import tileworld.agent.Group7AgentBase;
import tileworld.environment.TWEntity;
import tileworld.environment.TWHole;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;

/**
 * Object categories tracked by AgentHanny's memory sidecar and hazard model.
 */
public enum MemoryObjectType {
    TILE("T"),
    HOLE("H"),
    OBSTACLE("O");

    private final String shortCode;

    MemoryObjectType(String shortCode) {
        this.shortCode = shortCode;
    }

    public String shortCode() {
        return shortCode;
    }

    public static MemoryObjectType fromEntity(TWEntity entity) {
        if (entity instanceof TWTile) {
            return TILE;
        }
        if (entity instanceof TWHole) {
            return HOLE;
        }
        if (entity instanceof TWObstacle) {
            return OBSTACLE;
        }
        return null;
    }

    public static MemoryObjectType fromCommType(Group7AgentBase.CommType type) {
        if (type == Group7AgentBase.CommType.OBS_NEW_TILE) {
            return TILE;
        }
        if (type == Group7AgentBase.CommType.OBS_NEW_HOLE) {
            return HOLE;
        }
        if (type == Group7AgentBase.CommType.OBS_OBSTACLE) {
            return OBSTACLE;
        }
        return null;
    }

    public static MemoryObjectType fromSnapshotCode(String code) {
        if ("T".equals(code)) {
            return TILE;
        }
        if ("H".equals(code)) {
            return HOLE;
        }
        if ("O".equals(code)) {
            return OBSTACLE;
        }
        return null;
    }
}
