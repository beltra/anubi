# Optional LIO through reliable EKF input

DLIO is optional and feeds the navigation stack through `/lidar_odom_reliable`, not directly through raw `/lidar_odom` or direct TF ownership. The relay injects usable covariance before EKF fusion, and the EKF remains the owner of `odom -> base_footprint` for both simulation and real hardware. This avoids over-trusting vendor odometry outputs and keeps Nav2 on the fused odometry contract.
