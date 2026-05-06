#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "go2_bringup/sport_client.hpp"

namespace
{

TEST(SportClientTest, RecoveryStandUsesExpectedApiId)
{
  go2_bringup::SportClient client;
  unitree_api::msg::Request req;

  client.RecoveryStand(req);

  EXPECT_EQ(req.header.identity.api_id, go2_bringup::SPORT_API_RECOVERYSTAND);
}

TEST(SportClientTest, MoveEncodesVelocityTriplet)
{
  go2_bringup::SportClient client;
  unitree_api::msg::Request req;

  client.Move(req, 0.4f, -0.2f, 0.6f);

  const auto json = nlohmann::json::parse(req.parameter);
  EXPECT_EQ(req.header.identity.api_id, go2_bringup::SPORT_API_MOVE);
  EXPECT_FLOAT_EQ(json.at("x").get<float>(), 0.4f);
  EXPECT_FLOAT_EQ(json.at("y").get<float>(), -0.2f);
  EXPECT_FLOAT_EQ(json.at("z").get<float>(), 0.6f);
}

TEST(SportClientTest, ClassicWalkEncodesEnableFlag)
{
  go2_bringup::SportClient client;
  unitree_api::msg::Request req;

  client.ClassicWalk(req, true);

  const auto json = nlohmann::json::parse(req.parameter);
  EXPECT_EQ(req.header.identity.api_id, go2_bringup::SPORT_API_CLASSICWALK);
  EXPECT_TRUE(json.at("data").get<bool>());
}

TEST(SportClientTest, StopMoveUsesExpectedApiId)
{
  go2_bringup::SportClient client;
  unitree_api::msg::Request req;

  client.StopMove(req);

  EXPECT_EQ(req.header.identity.api_id, go2_bringup::SPORT_API_STOPMOVE);
}

}  // namespace
