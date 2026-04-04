package tileworld.agent;

import sim.util.Int2D;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;

/**
 * AgentMinimal
 *
 * Minimal baseline agent built on Group7AgentBase.
 * Policy:
 * 1) Refuel conservatively when needed.
 * 2) Immediate PICKUP/PUTDOWN if standing on valid object.
 * 3) If carrying tiles, go to nearest known hole.
 * 4) Else go to nearest known tile.
 * 5) Fallback to zig-zag exploration.
 */
public class AgentMinimal extends Group7AgentBase {
    private static final int FUEL_MARGIN = 20;

    public AgentMinimal(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
    }

    @Override
    protected TWThought think() {
        rememberFuelStationsInSensorRange();

        // Refuel now if we are on the station and fuel is low.
        if (this.getEnvironment().inFuelStation(this) && shouldRefuel(FUEL_MARGIN)) {
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }

        // Head to station when fuel gets tight.
        if (shouldRefuel(FUEL_MARGIN)) {
            TWFuelStation station = findFuelStationInMemory();
            if (station != null) {
                return stepToward(new Int2D(station.getX(), station.getY()));
            }
            return exploreZigZag();
        }

        // Opportunistic actions on current cell.
        TWEntity here = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
        if (here instanceof TWHole && this.hasTile()) {
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (here instanceof TWTile && this.carriedTiles.size() < 3) {
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        // Goal selection from memory.
        if (this.hasTile()) {
            TWHole hole = nearestKnownHole();
            if (hole != null) {
                return stepToward(new Int2D(hole.getX(), hole.getY()));
            }
        } else {
            TWTile tile = nearestKnownTile();
            if (tile != null) {
                return stepToward(new Int2D(tile.getX(), tile.getY()));
            }
        }

        return exploreZigZag();
    }
}
