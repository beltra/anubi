# Dynamic Obstacles

This context defines moving-obstacle language from the `feat/dynObstacles` branch.
It is branch-scoped until the runtime nodes and interfaces are merged into `feat/sim2real`.

## Language

**Dynamic Obstacle Pipeline**:
The moving-obstacle perception chain from world-frame points through clusters, tracks, and predicted occupancy.
_Avoid_: Obstacle detector

**World-Frame Obstacle Points**:
LiDAR points transformed into `map` at the cloud timestamp before clustering.
_Avoid_: Moving points

**Obstacle Cluster**:
A spatial group of obstacle points that may become a tracked obstacle.
_Avoid_: Blob

**Obstacle Track**:
A persistent moving-object estimate with position and velocity in the world frame.
_Avoid_: Target

**Predicted Occupancy**:
A costmap-compatible projection of current obstacle tracks into near-future occupied space.
_Avoid_: Future map

**Stale Occupancy Clear**:
The requirement that predicted occupancy becomes empty immediately when current tracks disappear.
_Avoid_: Timeout cleanup

## Relationships

- **World-Frame Obstacle Points** produce **Obstacle Clusters**.
- **Obstacle Clusters** update **Obstacle Tracks**.
- **Obstacle Tracks** produce **Predicted Occupancy**.
- **Stale Occupancy Clear** prevents old **Predicted Occupancy** from blocking navigation after movers disappear.
- Obstacle velocity is already world-frame after the cloud transform; downstream tracking must not subtract robot motion again.

## Example Dialogue

> **Dev:** "The track velocity seems too high; should we subtract robot velocity in the tracker?"
> **Domain expert:** "No. The inputs are **World-Frame Obstacle Points**, so **Obstacle Track** velocity is already expressed in the world frame."

## Flagged Ambiguities

- "dynamic obstacle" can mean a point, cluster, track, or occupancy projection; use the stage-specific term when debugging the pipeline.
