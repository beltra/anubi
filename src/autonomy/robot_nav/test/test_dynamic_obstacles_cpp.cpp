#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "robot_nav/dynamic_obstacles/common.hpp"

namespace
{

using robot_nav::dynamic_obstacles::Point2;
using robot_nav::dynamic_obstacles::Point3;
using robot_nav::dynamic_obstacles::oriented_footprint_points;
using robot_nav::dynamic_obstacles::points_in_shadow_cones;
using robot_nav::dynamic_obstacles::shadow_cone_from_obstacle;
using robot_nav::dynamic_obstacles::track_is_publishable;
using robot_nav::dynamic_obstacles::velocity_matches_ego_motion;
using robot_nav::dynamic_obstacles::yaw_from_quaternion;
using robot_nav::dynamic_obstacles::yaw_to_quaternion;

std::pair<double, double> span(const std::vector<Point3> & points)
{
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto & point : points) {
    min_x = std::min(min_x, point.x());
    min_y = std::min(min_y, point.y());
    max_x = std::max(max_x, point.x());
    max_y = std::max(max_y, point.y());
  }
  return {max_x - min_x, max_y - min_y};
}

TEST(DynamicObstaclesCommonTest, YawQuaternionRoundTrip)
{
  const double yaw = 1.35;
  EXPECT_NEAR(yaw_from_quaternion(yaw_to_quaternion(yaw)), yaw, 1.0e-9);
}

TEST(DynamicObstaclesCommonTest, OrientedFootprintUsesBoxExtents)
{
  const auto points = oriented_footprint_points(0.0, 0.0, 0.0, 2.8, 1.2, 0.2, 0.45, 0.1);
  const auto [span_x, span_y] = span(points);

  EXPECT_GE(span_x, 2.9);
  EXPECT_LE(span_x, 3.0);
  EXPECT_GE(span_y, 1.3);
  EXPECT_LE(span_y, 1.4);
  for (const auto & point : points) {
    EXPECT_DOUBLE_EQ(point.z(), 0.45);
  }
}

TEST(DynamicObstaclesCommonTest, OrientedFootprintRotatesPoints)
{
  const auto points = oriented_footprint_points(
    1.0, -2.0, M_PI / 2.0, 1.0, 0.4, 0.1, 0.2, 0.0);
  const auto [span_x, span_y] = span(points);

  EXPECT_LT(span_x, span_y);

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const auto & point : points) {
    mean_x += point.x();
    mean_y += point.y();
  }
  mean_x /= static_cast<double>(points.size());
  mean_y /= static_cast<double>(points.size());

  EXPECT_NEAR(mean_x, 1.0, 1.0e-9);
  EXPECT_NEAR(mean_y, -2.0, 1.0e-9);
}

TEST(DynamicObstaclesCommonTest, ShadowConeMasksOnlyOccludedPoints)
{
  bool valid = false;
  const auto cone = shadow_cone_from_obstacle(
    Point2(0.0, 0.0),
    Point2(2.0, 0.0),
    Point2(0.6, 0.6),
    10.0,
    0.02,
    0.10,
    6.0,
    0.5,
    &valid);

  ASSERT_TRUE(valid);
  const std::vector<Point3> points{
    Point3(2.2, 0.0, 0.5),
    Point3(3.4, 0.05, 0.5),
    Point3(3.4, 1.5, 0.5),
  };

  const auto mask = points_in_shadow_cones(points, {cone});
  ASSERT_EQ(mask.size(), 3U);
  EXPECT_FALSE(mask[0]);
  EXPECT_TRUE(mask[1]);
  EXPECT_FALSE(mask[2]);
  EXPECT_NEAR(cone.expires_at, 10.5, 1.0e-9);
}

TEST(DynamicObstaclesCommonTest, ShadowConeMasksMostOfFarCluster)
{
  bool valid = false;
  const auto cone = shadow_cone_from_obstacle(
    Point2(0.0, 0.0),
    Point2(2.0, 0.0),
    Point2(0.8, 0.6),
    5.0,
    0.06,
    0.15,
    8.0,
    0.75,
    &valid);

  ASSERT_TRUE(valid);
  const std::vector<Point3> far_cluster{
    Point3(3.1, 0.00, 0.5),
    Point3(3.3, 0.08, 0.5),
    Point3(3.6, -0.10, 0.5),
    Point3(3.5, 0.12, 0.5),
    Point3(3.2, 1.20, 0.5),
  };

  const auto mask = points_in_shadow_cones(far_cluster, {cone});
  int shadowed = 0;
  for (bool value : mask) {
    shadowed += value ? 1 : 0;
  }
  EXPECT_GE(shadowed, 4);
}

TEST(DynamicObstaclesCommonTest, VelocityMatchesEgoMotionRequiresDirectionAndRatio)
{
  const Point2 ego(0.4, 0.0);

  EXPECT_TRUE(velocity_matches_ego_motion(
      Point2(0.35, 0.02), ego, 0.08, 0.35, 1.8, 0.75));
  EXPECT_TRUE(velocity_matches_ego_motion(
      Point2(-0.35, -0.02), ego, 0.08, 0.35, 1.8, 0.75));
  EXPECT_FALSE(velocity_matches_ego_motion(
      Point2(0.0, 0.35), ego, 0.08, 0.35, 1.8, 0.75));
  EXPECT_FALSE(velocity_matches_ego_motion(
      Point2(1.2, 0.0), ego, 0.08, 0.35, 1.8, 0.75));
}

TEST(DynamicObstaclesCommonTest, TrackPublishRequiresRecentConfidentObservations)
{
  EXPECT_TRUE(track_is_publishable(4, 0, 4, 0, 0.75, 0.6));
  EXPECT_FALSE(track_is_publishable(4, 1, 4, 0, 0.75, 0.6));
  EXPECT_FALSE(track_is_publishable(4, 0, 4, 0, 0.50, 0.6));
}

}  // namespace
