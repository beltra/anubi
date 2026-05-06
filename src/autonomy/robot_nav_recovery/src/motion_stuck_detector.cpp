#include "robot_nav_recovery/motion_stuck_detector.hpp"

#include <algorithm>
#include <cmath>

namespace robot_nav_recovery
{
namespace
{

double planarSpeed(double x, double y)
{
  return std::hypot(x, y);
}

}  // namespace

MotionStuckDetector::MotionStuckDetector(StuckConfig config)
: config_(config)
{
}

bool MotionStuckDetector::update(const MotionSample & sample)
{
  if (!commandedMotion(sample) || observedMotion(sample)) {
    reset();
    return false;
  }

  if (!blocked_since_s_.has_value()) {
    blocked_since_s_ = sample.time_s;
    stuck_ = false;
    return false;
  }

  stuck_ = sample.time_s - blocked_since_s_.value() >= config_.stuck_duration_s;
  return stuck_;
}

bool MotionStuckDetector::stuck() const
{
  return stuck_;
}

void MotionStuckDetector::reset()
{
  blocked_since_s_.reset();
  stuck_ = false;
}

bool MotionStuckDetector::commandedMotion(const MotionSample & sample) const
{
  return planarSpeed(sample.cmd_linear_x, sample.cmd_linear_y) >= config_.cmd_speed_threshold_mps ||
         std::fabs(sample.cmd_angular_z) >= config_.cmd_speed_threshold_mps;
}

bool MotionStuckDetector::observedMotion(const MotionSample & sample) const
{
  return planarSpeed(sample.odom_linear_x, sample.odom_linear_y) >= config_.odom_speed_threshold_mps ||
         std::fabs(sample.odom_angular_z) >= config_.odom_speed_threshold_mps;
}

}  // namespace robot_nav_recovery
