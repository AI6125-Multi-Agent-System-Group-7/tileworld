package tileworld.agent;

import java.util.ArrayList;
import java.util.List;

import sim.field.grid.ObjectGrid2D;
import sim.util.Int2D;
import tileworld.Parameters;
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
    private static final int FUEL_MARGIN = 40;
    private static final int CRITICAL_FUEL_WITHOUT_STATION = 60;

    public AgentMinimal(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
    }

    @Override
    protected TWThought think() {
        rememberFuelStationsInSensorRange();
        rememberFuelStationFromMessages();

        TWFuelStation station = findFuelStationInMemory();

        // Top up whenever we are already on station.
        if (this.getEnvironment().inFuelStation(this) && this.getFuelLevel() < Parameters.defaultFuelLevel) {
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }

        // Head to station when fuel gets tight, otherwise preserve fuel if station unknown.
        if (shouldRefuel(FUEL_MARGIN)) {
            if (station != null) {
                return stepToward(new Int2D(station.getX(), station.getY()));
            }
            if (this.getFuelLevel() <= CRITICAL_FUEL_WITHOUT_STATION) {
                clearPlan();
                return waitThought();
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
            TWHole hole = selectHoleTarget();
            if (hole != null) {
                return stepToward(new Int2D(hole.getX(), hole.getY()));
            }
        } else {
            TWTile tile = selectTileTarget();
            if (tile != null) {
                return stepToward(new Int2D(tile.getX(), tile.getY()));
            }
        }

        return exploreZigZag();
    }

    private void rememberFuelStationFromMessages() {
        List<Message> inbox = new ArrayList<Message>(this.getEnvironment().getMessages());
        ObjectGrid2D memoryGrid = this.memory.getMemoryGrid();

        for (Message message : inbox) {
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
            if (parsed.type != CommType.OBS_FUEL_ONCE) {
                continue;
            }

            if (!(memoryGrid.get(parsed.x, parsed.y) instanceof TWFuelStation)) {
                memoryGrid.set(parsed.x, parsed.y, new TWFuelStation(parsed.x, parsed.y, this.getEnvironment()));
            }
        }
    }

    private TWTile selectTileTarget() {
        TWTile visible = closestVisibleTile();
        if (visible != null) {
            return visible;
        }
        return sanitizeTile(nearestKnownTile());
    }

    private TWHole selectHoleTarget() {
        TWHole visible = closestVisibleHole();
        if (visible != null) {
            return visible;
        }
        return sanitizeHole(nearestKnownHole());
    }

    private TWTile sanitizeTile(TWTile candidate) {
        if (candidate == null) {
            return null;
        }
        if (!isInSensorRange(candidate.getX(), candidate.getY())) {
            return candidate;
        }

        TWEntity observed = (TWEntity) this.getEnvironment().getObjectGrid().get(candidate.getX(), candidate.getY());
        if (observed instanceof TWTile) {
            return candidate;
        }

        forgetCell(candidate.getX(), candidate.getY());
        return null;
    }

    private TWHole sanitizeHole(TWHole candidate) {
        if (candidate == null) {
            return null;
        }
        if (!isInSensorRange(candidate.getX(), candidate.getY())) {
            return candidate;
        }

        TWEntity observed = (TWEntity) this.getEnvironment().getObjectGrid().get(candidate.getX(), candidate.getY());
        if (observed instanceof TWHole) {
            return candidate;
        }

        forgetCell(candidate.getX(), candidate.getY());
        return null;
    }

    private void forgetCell(int x, int y) {
        this.memory.removeAgentPercept(x, y);
        this.memory.getMemoryGrid().set(x, y, null);
        clearPlan();
    }

    private boolean isInSensorRange(int x, int y) {
        int dx = Math.abs(x - this.getX());
        int dy = Math.abs(y - this.getY());
        return Math.max(dx, dy) <= Parameters.defaultSensorRange;
    }
}
