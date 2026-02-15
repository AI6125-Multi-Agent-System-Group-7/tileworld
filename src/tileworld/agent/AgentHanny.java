package tileworld.agent;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;

/**
 * AgentHanny by HumanHanny
 *
 * A tiny, keep-alive agent. (for now~)
 * Policy is simple; decisions are not.
 */
public class AgentHanny extends Group7AgentBase {
    private static final int FUEL_MARGIN = 120; // when Agent don't know nothing about fuel station

    private Int2D wanderTarget;
    private int stepCount;

    public AgentHanny(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
    }

    @Override
    public void communicate() {
        // Share what we see so future teammates can pretend they also saw it.
        rememberFuelStationsInSensorRange();

        StringBuilder sb = new StringBuilder();
        sb.append("pos=").append(this.getX()).append(",").append(this.getY());
        sb.append(" fuel=").append((int) this.getFuelLevel());

        TWTile tile = closestVisibleTile();
        if (tile != null) {
            sb.append(" tile=").append(tile.getX()).append(",").append(tile.getY());
        }

        TWHole hole = closestVisibleHole();
        if (hole != null) {
            sb.append(" hole=").append(hole.getX()).append(",").append(hole.getY());
        }

        TWFuelStation station = findFuelStationInMemory();
        if (station != null) {
            sb.append(" fuelStation=").append(station.getX()).append(",").append(station.getY());
        }

        Message message = new Message(getName(), "ALL", sb.toString());
        this.getEnvironment().receiveMessage(message);
    }

    @Override
    protected TWThought think() {
        rememberFuelStationsInSensorRange();

        // If we are already on the fuel station and low, horay, instant grafitication!
        if (this.getEnvironment().inFuelStation(this) && shouldRefuel(FUEL_MARGIN)) {
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }

        // If fuel is getting spicy, go find the station.
        if (shouldRefuel(FUEL_MARGIN)) {
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
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (here instanceof TWTile && this.carriedTiles.size() < 3) {
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        // If carrying tiles, go fill a hole.
        if (this.hasTile()) {
            TWHole hole = nearestKnownHole();
            if (hole != null) {
                if (targetVisibleButMissing(hole)) {
                    this.memory.removeObject(hole);
                    clearPlan();
                } else {
                    return stepToward(new Int2D(hole.getX(), hole.getY()));
                }
            }
        }

        // If we can carry more, go grab a tile.
        if (this.carriedTiles.size() < 3) {
            TWTile tile = nearestKnownTile();
            if (tile != null) {
                if (targetVisibleButMissing(tile)) {
                    this.memory.removeObject(tile);
                    clearPlan();
                } else {
                    return stepToward(new Int2D(tile.getX(), tile.getY()));
                }
            }
        }

        // Otherwise, explore like 'Man vs Wild' host Bear Grylls.
        return explore();
    }

    @Override
    protected void act(TWThought thought) {
        stepCount++;
        String action = (thought == null) ? "NULL" : thought.getAction().name();
        String dir = (thought != null && thought.getAction() == TWAction.MOVE)
                ? thought.getDirection().name()
                : "-";
        System.out.println(String.format(
                "[Hanny step %d] %s %s | pos=(%d,%d) fuel=%d score=%d",
                stepCount,
                action,
                dir,
                this.getX(),
                this.getY(),
                (int) this.getFuelLevel(),
                this.getScore()));
        super.act(thought);
    }

    private TWThought explore() {
        if (wanderTarget == null || at(wanderTarget)) {
            wanderTarget = pickRandomNearbyTarget();
        }
        return stepToward(wanderTarget);
    }

    private boolean at(Int2D pos) {
        return pos != null && this.getX() == pos.x && this.getY() == pos.y;
    }

    private Int2D pickRandomNearbyTarget() {
        int radius = Parameters.defaultSensorRange * 2 + 2;
        for (int i = 0; i < 24; i++) {
            int dx = this.getEnvironment().random.nextInt(radius * 2 + 1) - radius;
            int dy = this.getEnvironment().random.nextInt(radius * 2 + 1) - radius;
            if (dx == 0 && dy == 0) {
                continue;
            }
            int x = this.getX() + dx;
            int y = this.getY() + dy;
            if (this.getEnvironment().isInBounds(x, y) && !isBlocked(x, y)) {
                return new Int2D(x, y);
            }
        }

        // Fall back to any random valid location if nearby picks fail.
        for (int i = 0; i < 50; i++) {
            int x = this.getEnvironment().random.nextInt(this.getEnvironment().getxDimension());
            int y = this.getEnvironment().random.nextInt(this.getEnvironment().getyDimension());
            if (this.getEnvironment().isInBounds(x, y) && !isBlocked(x, y)) {
                return new Int2D(x, y);
            }
        }

        return null;
    }

    private boolean targetVisibleButMissing(TWEntity target) {
        if (target == null) {
            return false;
        }
        int dx = Math.abs(target.getX() - this.getX());
        int dy = Math.abs(target.getY() - this.getY());
        int chebyshev = Math.max(dx, dy);

        if (chebyshev > Parameters.defaultSensorRange) {
            return false; // not visible, so we are not allowed to judge it yet
        }

        TWEntity here = (TWEntity) this.getEnvironment().getObjectGrid().get(target.getX(), target.getY());
        if (target instanceof TWTile) {
            return !(here instanceof TWTile);
        }
        if (target instanceof TWHole) {
            return !(here instanceof TWHole);
        }
        return false;
    }
}
