# Agent 3 - AgentSun

AgentSun is a regional explorer agent built on `Group7AgentBase`. It is designed around one idea: **cover the map systematically, act on what you see, and never run out of fuel**. The agent does not model object lifetimes probabilistically; instead, it records the simulation step at which it personally first observed each tile or hole, and uses that timestamp to decide whether a remembered target is still worth chasing before the object expires.

## 1. Think Phase and Decision Priority

`think()` runs a strict four-level priority chain every step:

- **Step 0 — Housekeeping**: Update fuel-station knowledge (`rememberFuelStationsInSensorRange` + `findFuelStationInMemory`), run sensor-based memory verification, parse teammate messages.
- **Priority 1 — Fuel**: On the fuel station and low? Refuel now. Fuel critically low relative to station distance? Navigate to station immediately. Near the station and fuel below threshold? Detour to refuel — unless a hole is visible and tiles are being carried (high-value delivery takes priority).
- **Priority 2 — Immediate cell action**: Standing on a hole while carrying tiles → PUTDOWN. Standing on a tile with carry space → PICKUP. No pathfinding needed.
- **Priority 3 — Task completion**: If carrying tiles, find the nearest unclaimed hole (visible > memory). If carry space remains, find the nearest unclaimed tile (visible > memory), subject to fuel-safety and expiry-reachability checks.
- **Priority 4 — Regional exploration**: Navigate through a pre-computed boustrophedon (lawn-mower) sweep of anchor points. Advance to the next anchor on arrival.

Every movement decision is routed through `ensureNotStuck()` before being returned.

## 2. Reused Functions from `Group7AgentBase`

| Function | Role in AgentSun |
|----------|-----------------|
| `stepToward(Int2D)` | Primary A\* navigation to tiles, holes, fuel station, and region anchors |
| `clearPlan()` | Called whenever the A\* path blocks and the agent falls back to greedy movement |
| `exploreZigZag()` | Last-resort fallback when both A\* and greedy fail for an anchor point |
| `rememberFuelStationsInSensorRange()` | Called every step to persist fuel-station sightings into working memory |
| `findFuelStationInMemory()` | Used to initialise `fuelStationPosition` on first discovery |
| `shouldRefuel(int margin)` | Not directly used; replaced by the custom `isCriticalFuelLevel()` |
| `closestVisibleTile()` / `closestVisibleHole()` | Real-time sensor queries for task selection |
| `parseProtocolMessage(Message)` | Parses teammate messages in the G7P1 protocol |
| `communicate()` | Broadcasts tile/hole/obstacle observations and pickup/fill action events (auto, not overridden) |
| `act(TWThought)` | Executes MOVE/PICKUP/PUTDOWN/REFUEL; called via `super.act()` after logging |

## 3. Exploration Strategy: Boustrophedon Anchor Sweep

Instead of a small set of fixed corners, AgentSun generates a **boustrophedon (lawn-mower) grid of anchor points** at initialisation. Strip spacing equals one sensor diameter (`2 × sensorRange + 1 = 7` cells), which guarantees roughly 85% map coverage with no gaps between adjacent strips.

```
Strip 0 (→):  (left, 4)  → (right, 4)
Strip 1 (←):  (right, 11) → (left, 11)
Strip 2 (→):  (left, 18) → (right, 18)
...
```

- Anchors alternate left-to-right then right-to-left, minimising total travel distance between strips.
- The bottom strip is added explicitly when the step size does not divide the map height evenly.
- A jitter of ±`sensorRange` cells is applied to each anchor (once per anchor, not every step) to reduce path-planning collisions with obstacles.
- On reaching an anchor (within `sensorRange` of the target), the agent advances to the next anchor in the list, cycling back to the start when the sweep is complete.

## 4. Scoring Strategy

AgentSun scores by completing the **tile → carry → hole → drop** cycle.

**Tile collection:**
1. Check `closestVisibleTile()` (ground truth, within sensor range).
2. Fall back to `memory.getNearbyTile(x, y, MEMORY_EXPIRY)` for recently remembered tiles.
3. Skip tiles claimed by teammates (via `claimedTargets`).
4. Skip tiles where `isFuelSafeForDetour()` fails (not enough fuel to reach the tile and then reach the station).
5. Skip tiles where `isReachableBeforeExpiry()` fails (the tile will have expired before the agent can arrive).

**Hole delivery:**
1. Check `closestVisibleHole()` first.
2. Fall back to `memory.getNearbyHole(x, y, MEMORY_EXPIRY)`, filtered by `isReachableBeforeExpiry()`.
3. Skip holes claimed by teammates.
4. No fuel-safety gate on holes — delivering a tile already in hand is always worth doing.

**Immediate actions** (standing directly on a tile or hole) are handled before any pathfinding, so the agent never wastes a step moving away from an object underfoot.

## 5. Fuel Management Strategy

Three-tier strategy, in priority order:

**Tier 1 — On-station refuel**: If the agent is standing on the fuel station and `shouldRefuel(CRITICAL_FUEL_MARGIN)` is true, execute REFUEL immediately.

**Tier 2 — Critical navigation**: `isCriticalFuelLevel()` returns true when:
```
fuelLevel <= manhattan(pos, station) + CRITICAL_FUEL_MARGIN (20)
```
If triggered, the agent cancels all other tasks and navigates to the station. If A\* fails, greedy movement takes over.

**Tier 3 — Opportunistic refuel**: Triggered when:
```
fuelLevel < OPPORTUNISTIC_FUEL_THRESHOLD (150)  AND
manhattan(pos, station) < NEAR_STATION_DISTANCE (8)
```
Exception: if the agent is carrying a tile and a hole is currently visible, the delivery is completed first before refuelling.

**Fuel-safe detour check** (before pursuing any tile):
```
fuelLevel > distToTile + distTileToStation + CRITICAL_FUEL_MARGIN
```
This ensures the agent always has enough fuel to complete the pickup and return to the station.

**Fuel station discovery**: `rememberFuelStationsInSensorRange()` is called every step. The station position is also learned from teammate `OBS_FUEL_ONCE` messages. Once known, the position is cached in `fuelStationPosition` for O(1) distance checks.

Compared to the previous version (RegionalExplorerAgent), all three thresholds have been tightened for larger maps: `CRITICAL_FUEL_MARGIN` dropped from 30 → 20, `OPPORTUNISTIC_FUEL_THRESHOLD` from 250 → 150, and `NEAR_STATION_DISTANCE` from 15 → 8, reducing unnecessary refuel detours.

## 6. Sensor-Based Memory Verification

Each step, AgentSun scans every cell within sensor range and cross-checks the working memory against ground truth:

- If memory records a tile, hole, or obstacle at a cell that the sensor now shows as empty, the memory entry is cleared immediately.
- This replaces the previous time-based `cleanExpiredPercepts()` approach, which was imprecise and could leave ghost obstacles blocking A\* for up to `lifeTime` steps.
- Fuel stations are exempt — they never expire.

This is implemented in `updateObservationTimes()`, which also records the simulation step (`cellObservedAt`) at which each tile or hole was first personally observed.

## 7. Expiry-Aware Target Filtering

Before committing to a remembered tile or hole, `isReachableBeforeExpiry()` estimates whether the object will still be alive when the agent arrives:

```
age + manhattan(pos, target) + FRESHNESS_BUFFER(5) < MEMORY_EXPIRY(30)
```

- `age` = current step − step at which the agent first personally saw the object.
- If no personal observation record exists (the target was learned from a teammate message), the check conservatively returns `true`.
- This prevents the agent from wasting movement chasing targets that will have expired on arrival, which is particularly important in Configuration Two where `lifeTime = 30`.

## 8. Anti-Stuck Mechanisms

All movement decisions pass through `ensureNotStuck()`:

1. If the returned thought is a real move (not MOVE Z), reset `stuckCounter` and return it.
2. If MOVE Z: increment `stuckCounter`, call `clearPlan()`, then try `greedyStepToward(currentTarget)`.
3. If greedy also fails: try `tryRandomMove()` — a random walkable direction selected from {N, S, E, W}.
4. If all fail: return MOVE Z as the last resort.

`greedyStepToward()` bypasses A\* entirely and checks only the real environment (`getEnvironment().isCellBlocked()`), not working memory. This avoids stale-obstacle deadlocks that cause A\* to return no path.

## 9. Team Coordination

AgentSun processes three message types from teammates each step:

| Message | Action |
|---------|--------|
| `ACTION_PICKUP_TILE` | Add to `claimedTargets`; remove from memory and `cellObservedAt` |
| `ACTION_FILL_HOLE` | Add to `claimedTargets`; remove from memory and `cellObservedAt` |
| `OBS_FUEL_ONCE` | Cache fuel station position if not yet known |

`claimedTargets` is cleared and rebuilt every step from the current message queue (messages do not persist across steps). Before pursuing any tile or hole, the agent checks `isClaimed(x, y)`.

Outbound communication is handled entirely by `Group7AgentBase.communicate()` — AgentSun does not override it.

## 10. Key Constants

| Constant | Value | Meaning |
|----------|-------|---------|
| `CRITICAL_FUEL_MARGIN` | 20 | Safety buffer for critical-fuel calculation |
| `OPPORTUNISTIC_FUEL_THRESHOLD` | 150 | Below this fuel level, opportunistic refuel may trigger |
| `NEAR_STATION_DISTANCE` | 8 | Max distance from station for opportunistic refuel |
| `MEMORY_EXPIRY` | `Parameters.lifeTime` | Age limit for working-memory entries (= 30 in Config Two) |
| `FRESHNESS_BUFFER` | 5 | Extra steps added to expiry estimate as a safety margin |

---

## 11. Known Issue: Fuel Exhaustion in Configuration Two

Under **Configuration Two** (80×80 map, `lifeTime=30`, `µ=2`, `σ=0.5`), AgentSun has been observed running out of early in the simulation.

