# Dynamic obstacle pipeline branch

The `feat/dynObstacles` branch models moving obstacles as a pipeline from world-frame points to clusters, tracks, and predicted occupancy. The design keeps obstacle velocity in the world frame after timestamped map transforms, publishes predicted occupancy for Nav2, and clears occupancy immediately when current tracks disappear. Until the branch is merged into `feat/sim2real`, its context remains branch-scoped even though the default Nav2 profile references `/dynamic_obstacles/predicted_occupancy`.
