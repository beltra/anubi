#pragma once

#include <optional>

namespace robot_nav_recovery
{

struct StuckConfig
{
  double cmd_speed_threshold_mps{0.04};
  double odom_speed_threshold_mps{0.015};
  double stuck_duration_s{2.0};
};

struct MotionSample
{
  double time_s{0.0};
  double cmd_linear_x{0.0};
  double cmd_linear_y{0.0};
  double cmd_angular_z{0.0};
  double odom_linear_x{0.0};
  double odom_linear_y{0.0};
  double odom_angular_z{0.0};
};

class MotionStuckDetector
{
public:
  explicit MotionStuckDetector(StuckConfig config);

  bool update(const MotionSample & sample);
  bool stuck() const;
  void reset();

private:
  bool commandedMotion(const MotionSample & sample) const;
  bool observedMotion(const MotionSample & sample) const;

  StuckConfig config_;
  std::optional<double> blocked_since_s_;
  bool stuck_{false};
};

}  // namespace robot_nav_recovery
