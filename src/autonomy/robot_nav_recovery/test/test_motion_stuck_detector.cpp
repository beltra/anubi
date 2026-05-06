#include "robot_nav_recovery/motion_stuck_detector.hpp"

#include <gtest/gtest.h>

namespace robot_nav_recovery
{

TEST(MotionStuckDetector, DetectsCommandedMotionWithoutOdomProgress)
{
  StuckConfig config;
  config.cmd_speed_threshold_mps = 0.04;
  config.odom_speed_threshold_mps = 0.01;
  config.stuck_duration_s = 1.0;
  MotionStuckDetector detector(config);

  MotionSample sample;
  sample.cmd_linear_x = 0.1;

  sample.time_s = 0.0;
  EXPECT_FALSE(detector.update(sample));
  sample.time_s = 0.5;
  EXPECT_FALSE(detector.update(sample));
  sample.time_s = 1.1;
  EXPECT_TRUE(detector.update(sample));
  EXPECT_TRUE(detector.stuck());
}

TEST(MotionStuckDetector, OdomProgressResetsStuckWindow)
{
  StuckConfig config;
  config.stuck_duration_s = 1.0;
  MotionStuckDetector detector(config);

  MotionSample sample;
  sample.cmd_linear_x = 0.1;
  sample.time_s = 0.0;
  EXPECT_FALSE(detector.update(sample));

  sample.time_s = 1.1;
  sample.odom_linear_x = 0.05;
  EXPECT_FALSE(detector.update(sample));
  EXPECT_FALSE(detector.stuck());

  sample.time_s = 2.3;
  sample.odom_linear_x = 0.0;
  EXPECT_FALSE(detector.update(sample));
}

TEST(MotionStuckDetector, NoCommandDoesNotTriggerStuck)
{
  StuckConfig config;
  config.stuck_duration_s = 0.5;
  MotionStuckDetector detector(config);

  MotionSample sample;
  sample.time_s = 10.0;
  EXPECT_FALSE(detector.update(sample));
  sample.time_s = 11.0;
  EXPECT_FALSE(detector.update(sample));
}

}  // namespace robot_nav_recovery
