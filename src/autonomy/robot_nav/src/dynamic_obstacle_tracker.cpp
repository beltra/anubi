#include <algorithm>
#include <cstdint>
#include <cmath>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "robot_common_interfaces/msg/dynamic_obstacle_cluster_array.hpp"
#include "robot_common_interfaces/msg/dynamic_obstacle_track.hpp"
#include "robot_common_interfaces/msg/dynamic_obstacle_track_array.hpp"
#include "std_msgs/msg/header.hpp"

#include "robot_nav/dynamic_obstacles/common.hpp"

namespace robot_nav::dynamic_obstacles
{
namespace
{

struct TrackState
{
  int track_id{0};
  TrackVector x{TrackVector::Zero()};
  TrackMatrix p{TrackMatrix::Identity()};
  Point3 size{Point3::Zero()};
  int hits{1};
  int misses{0};
  int age{1};
  std::vector<bool> history{true};
  double last_stamp{0.0};
};

struct DistanceCandidate
{
  double distance{0.0};
  int track_id{0};
  std::size_t measurement_index{0};
};

}  // namespace

class DynamicObstacleTracker : public rclcpp::Node
{
public:
  DynamicObstacleTracker()
  : Node("dynamic_obstacle_tracker")
  {
    declare_parameter("input_topic", "/dynamic_obstacles/clusters");
    declare_parameter("output_topic", "/dynamic_obstacles/tracks");
    declare_parameter("publish_frequency", 10.0);
    declare_parameter("prediction_horizon_sec", 3.0);
    declare_parameter("prediction_step_sec", 0.3);
    declare_parameter("confirmation_hits", 2);
    declare_parameter("max_misses", 6);
    declare_parameter("max_publish_misses", 2);
    declare_parameter("min_publish_confidence", 0.5);
    declare_parameter("history_window", 8);
    declare_parameter("gating_threshold", 9.21);
    declare_parameter("initial_position_variance", 0.18);
    declare_parameter("initial_yaw_variance", 0.8);
    declare_parameter("initial_speed_variance", 0.6);
    declare_parameter("initial_yaw_rate_variance", 0.5);
    declare_parameter("measurement_variance", 0.08);
    declare_parameter("process_position_variance", 0.04);
    declare_parameter("process_yaw_variance", 0.08);
    declare_parameter("process_speed_variance", 0.20);
    declare_parameter("process_yaw_rate_variance", 0.20);
    declare_parameter("yaw_rate_epsilon", 0.05);
    declare_parameter("straight_line_prediction", true);
    declare_parameter("input_timeout_sec", 0.75);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    prediction_horizon_sec_ = get_parameter("prediction_horizon_sec").as_double();
    prediction_step_sec_ = get_parameter("prediction_step_sec").as_double();
    confirmation_hits_ = get_parameter("confirmation_hits").as_int();
    max_misses_ = get_parameter("max_misses").as_int();
    max_publish_misses_ = get_parameter("max_publish_misses").as_int();
    min_publish_confidence_ = get_parameter("min_publish_confidence").as_double();
    history_window_ = get_parameter("history_window").as_int();
    gating_threshold_ = get_parameter("gating_threshold").as_double();
    measurement_variance_ = get_parameter("measurement_variance").as_double();
    yaw_rate_epsilon_ = get_parameter("yaw_rate_epsilon").as_double();
    straight_line_prediction_ = get_parameter("straight_line_prediction").as_bool();
    input_timeout_sec_ = get_parameter("input_timeout_sec").as_double();

    initial_position_variance_ = get_parameter("initial_position_variance").as_double();
    initial_yaw_variance_ = get_parameter("initial_yaw_variance").as_double();
    initial_speed_variance_ = get_parameter("initial_speed_variance").as_double();
    initial_yaw_rate_variance_ = get_parameter("initial_yaw_rate_variance").as_double();

    process_q_ = TrackMatrix::Zero();
    process_q_(0, 0) = get_parameter("process_position_variance").as_double();
    process_q_(1, 1) = get_parameter("process_position_variance").as_double();
    process_q_(2, 2) = get_parameter("process_yaw_variance").as_double();
    process_q_(3, 3) = get_parameter("process_speed_variance").as_double();
    process_q_(4, 4) = get_parameter("process_yaw_rate_variance").as_double();

    publisher_ =
      create_publisher<robot_common_interfaces::msg::DynamicObstacleTrackArray>(output_topic_, 10);
    subscription_ =
      create_subscription<robot_common_interfaces::msg::DynamicObstacleClusterArray>(
      input_topic_,
      10,
      std::bind(&DynamicObstacleTracker::clusters_callback, this, std::placeholders::_1));

    const double publish_frequency =
      std::max(1.0, get_parameter("publish_frequency").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_frequency),
      std::bind(&DynamicObstacleTracker::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "dynamic_obstacle_tracker: %s -> %s",
      input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  void predict_track(TrackState & track, double stamp) const
  {
    const double dt = stamp - track.last_stamp;
    if (dt <= 1.0e-4) {
      return;
    }

    const TrackMatrix f = ctrv_jacobian(track.x, dt, yaw_rate_epsilon_);
    track.x = ctrv_predict_state(track.x, dt, yaw_rate_epsilon_);
    track.p = f * track.p * f.transpose() + process_q_ * std::max(dt, 1.0e-3);
    track.last_stamp = stamp;
    track.age += 1;
  }

  void new_track(
    const robot_common_interfaces::msg::DynamicObstacleCluster & cluster,
    double stamp)
  {
    const double speed = std::hypot(cluster.velocity.x, cluster.velocity.y);
    double yaw = cluster.yaw;
    if (speed > 1.0e-3) {
      yaw = std::atan2(cluster.velocity.y, cluster.velocity.x);
    }

    TrackState track;
    track.track_id = next_track_id_;
    track.x <<
      cluster.centroid.x,
      cluster.centroid.y,
      yaw,
      speed,
      0.0;
    track.p = TrackMatrix::Zero();
    track.p(0, 0) = initial_position_variance_;
    track.p(1, 1) = initial_position_variance_;
    track.p(2, 2) = initial_yaw_variance_;
    track.p(3, 3) = initial_speed_variance_;
    track.p(4, 4) = initial_yaw_rate_variance_;
    track.size = Point3(cluster.size.x, cluster.size.y, cluster.size.z);
    track.last_stamp = stamp;
    tracks_[track.track_id] = track;
    next_track_id_ += 1;
  }

  std::vector<DistanceCandidate> distance_table(
    const std::vector<Point3> & measurements) const
  {
    Eigen::Matrix<double, 2, 5> h = Eigen::Matrix<double, 2, 5>::Zero();
    h(0, 0) = 1.0;
    h(1, 1) = 1.0;
    const Eigen::Matrix2d r = Eigen::Matrix2d::Identity() * measurement_variance_;

    std::vector<DistanceCandidate> distances;
    for (const auto & [track_id, track] : tracks_) {
      const Eigen::Matrix2d s = h * track.p * h.transpose() + r;
      const Eigen::Matrix2d inv_s = s.inverse();
      const Eigen::Vector2d predicted = h * track.x;
      for (std::size_t index = 0; index < measurements.size(); ++index) {
        const Eigen::Vector2d innovation = measurements[index].head<2>() - predicted;
        const double distance = innovation.transpose() * inv_s * innovation;
        if (distance <= gating_threshold_) {
          distances.push_back({distance, track_id, index});
        }
      }
    }

    std::sort(
      distances.begin(),
      distances.end(),
      [](const DistanceCandidate & lhs, const DistanceCandidate & rhs) {
        return lhs.distance < rhs.distance;
      });
    return distances;
  }

  void correct_track(
    TrackState & track,
    const robot_common_interfaces::msg::DynamicObstacleCluster & cluster,
    const Point3 & measurement)
  {
    Eigen::Matrix<double, 2, 5> h = Eigen::Matrix<double, 2, 5>::Zero();
    h(0, 0) = 1.0;
    h(1, 1) = 1.0;
    const Eigen::Matrix2d r = Eigen::Matrix2d::Identity() * measurement_variance_;

    const Eigen::Vector2d innovation = measurement.head<2>() - h * track.x;
    const Eigen::Matrix2d s = h * track.p * h.transpose() + r;
    const Eigen::Matrix<double, 5, 2> k = track.p * h.transpose() * s.inverse();

    track.x = track.x + k * innovation;
    track.x(2) = normalize_angle(track.x(2));
    track.p = (TrackMatrix::Identity() - k * h) * track.p;

    const double measured_speed = std::hypot(cluster.velocity.x, cluster.velocity.y);
    if (measured_speed > 1.0e-3) {
      const double measured_yaw = std::atan2(cluster.velocity.y, cluster.velocity.x);
      track.x(2) = normalize_angle(0.65 * track.x(2) + 0.35 * measured_yaw);
      track.x(3) = 0.7 * track.x(3) + 0.3 * measured_speed;
    }

    const Point3 measured_size(cluster.size.x, cluster.size.y, cluster.size.z);
    track.size = 0.8 * track.size + 0.2 * measured_size;
    track.hits += 1;
    track.misses = 0;
    track.history.push_back(true);
    if (static_cast<int>(track.history.size()) > history_window_) {
      track.history.erase(track.history.begin());
    }
  }

  void clusters_callback(
    const robot_common_interfaces::msg::DynamicObstacleClusterArray::SharedPtr msg)
  {
    const double stamp = stamp_seconds(msg->header.stamp);
    last_input_stamp_ = stamp;
    last_header_ = msg->header;

    for (auto & [track_id, track] : tracks_) {
      (void)track_id;
      predict_track(track, stamp);
    }

    std::vector<Point3> measurements;
    measurements.reserve(msg->clusters.size());
    for (const auto & cluster : msg->clusters) {
      measurements.emplace_back(cluster.centroid.x, cluster.centroid.y, cluster.centroid.z);
    }

    std::set<int> assigned_tracks;
    std::set<std::size_t> assigned_measurements;
    for (const auto & candidate : distance_table(measurements)) {
      if (
        assigned_tracks.count(candidate.track_id) > 0 ||
        assigned_measurements.count(candidate.measurement_index) > 0)
      {
        continue;
      }
      correct_track(
        tracks_.at(candidate.track_id),
        msg->clusters[candidate.measurement_index],
        measurements[candidate.measurement_index]);
      assigned_tracks.insert(candidate.track_id);
      assigned_measurements.insert(candidate.measurement_index);
    }

    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if (assigned_tracks.count(it->first) == 0) {
        it->second.misses += 1;
        it->second.history.push_back(false);
        if (static_cast<int>(it->second.history.size()) > history_window_) {
          it->second.history.erase(it->second.history.begin());
        }
        if (it->second.misses > max_misses_) {
          it = tracks_.erase(it);
          continue;
        }
      }
      ++it;
    }

    for (std::size_t index = 0; index < msg->clusters.size(); ++index) {
      if (assigned_measurements.count(index) == 0) {
        new_track(msg->clusters[index], stamp);
      }
    }

    publish(msg->header);
  }

  void timer_callback()
  {
    if (!last_header_.has_value()) {
      return;
    }

    const double now = get_clock()->now().seconds();
    if (last_input_stamp_.has_value() && now - *last_input_stamp_ > input_timeout_sec_) {
      tracks_.clear();
      robot_common_interfaces::msg::DynamicObstacleTrackArray out;
      out.header = *last_header_;
      const auto now_ns = get_clock()->now().nanoseconds();
      out.header.stamp.sec = static_cast<std::int32_t>(now_ns / 1000000000LL);
      out.header.stamp.nanosec = static_cast<std::uint32_t>(now_ns % 1000000000LL);
      publisher_->publish(out);
      return;
    }

    publish(*last_header_);
  }

  double track_confidence(const TrackState & track) const
  {
    const int hits = std::accumulate(
      track.history.begin(), track.history.end(), 0,
      [](int acc, bool value) {return acc + (value ? 1 : 0);});
    return static_cast<double>(hits) / static_cast<double>(std::max<std::size_t>(1U, track.history.size()));
  }

  robot_common_interfaces::msg::DynamicObstacleTrack track_msg(
    const TrackState & track,
    const std_msgs::msg::Header & header) const
  {
    robot_common_interfaces::msg::DynamicObstacleTrack msg;
    msg.id = static_cast<std::uint32_t>(track.track_id);
    msg.pose = pose_from_xy_yaw(track.x(0), track.x(1), 0.0, track.x(2));
    msg.velocity = vector_msg(Point3(
        track.x(3) * std::cos(track.x(2)),
        track.x(3) * std::sin(track.x(2)),
        0.0));
    msg.size = vector_msg(track.size);
    msg.speed = static_cast<float>(track.x(3));
    msg.yaw_rate = static_cast<float>(straight_line_prediction_ ? 0.0 : track.x(4));
    msg.confidence = static_cast<float>(track_confidence(track));

    TrackVector prediction = track.x;
    if (straight_line_prediction_) {
      prediction(4) = 0.0;
    }
    TrackMatrix covariance = track.p;
    double t = prediction_step_sec_;
    while (t <= prediction_horizon_sec_ + 1.0e-6) {
      const TrackMatrix f = ctrv_jacobian(prediction, prediction_step_sec_, yaw_rate_epsilon_);
      prediction = ctrv_predict_state(prediction, prediction_step_sec_, yaw_rate_epsilon_);
      covariance = f * covariance * f.transpose() + process_q_ * prediction_step_sec_;

      msg.predicted_poses.push_back(
        pose_from_xy_yaw(prediction(0), prediction(1), 0.0, prediction(2)));
      msg.predicted_stamps.push_back(stamp_plus_seconds(header.stamp, t));
      const double std_xy = std::sqrt(std::max(0.0, covariance(0, 0) + covariance(1, 1)));
      msg.predicted_position_std.push_back(static_cast<float>(std_xy));
      t += prediction_step_sec_;
    }
    return msg;
  }

  void publish(const std_msgs::msg::Header & header)
  {
    robot_common_interfaces::msg::DynamicObstacleTrackArray out;
    out.header = header;
    for (const auto & [track_id, track] : tracks_) {
      (void)track_id;
      if (track_is_publishable(
          track.hits,
          track.misses,
          static_cast<int>(confirmation_hits_),
          static_cast<int>(max_publish_misses_),
          track_confidence(track),
          min_publish_confidence_))
      {
        out.tracks.push_back(track_msg(track, header));
      }
    }
    publisher_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;

  double prediction_horizon_sec_{0.0};
  double prediction_step_sec_{0.0};
  long confirmation_hits_{0};
  long max_misses_{0};
  long max_publish_misses_{0};
  double min_publish_confidence_{0.0};
  long history_window_{0};
  double gating_threshold_{0.0};
  double measurement_variance_{0.0};
  double yaw_rate_epsilon_{0.0};
  bool straight_line_prediction_{true};
  double input_timeout_sec_{0.0};

  double initial_position_variance_{0.0};
  double initial_yaw_variance_{0.0};
  double initial_speed_variance_{0.0};
  double initial_yaw_rate_variance_{0.0};
  TrackMatrix process_q_{TrackMatrix::Identity()};

  std::map<int, TrackState> tracks_;
  int next_track_id_{1};
  std::optional<double> last_input_stamp_;
  std::optional<std_msgs::msg::Header> last_header_;

  rclcpp::Publisher<robot_common_interfaces::msg::DynamicObstacleTrackArray>::SharedPtr publisher_;
  rclcpp::Subscription<robot_common_interfaces::msg::DynamicObstacleClusterArray>::SharedPtr
    subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_nav::dynamic_obstacles

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_nav::dynamic_obstacles::DynamicObstacleTracker>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "dynamic_obstacle_tracker crashed: %s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
