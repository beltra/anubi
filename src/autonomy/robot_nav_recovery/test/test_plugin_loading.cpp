#include <dlfcn.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "nav2_core/behavior.hpp"
#include "pluginlib/class_loader.hpp"

namespace robot_nav_recovery
{

TEST(PluginLoading, BehaviorPluginsLoadThroughPluginlib)
{
  pluginlib::ClassLoader<nav2_core::Behavior> loader(
    "nav2_core", "nav2_core::Behavior");

  auto spin = loader.createSharedInstance("robot_nav_recovery/GuardedSpin");
  auto backup = loader.createSharedInstance("robot_nav_recovery/GuardedBackUp");
  auto lateral = loader.createSharedInstance("robot_nav_recovery/GuardedLateralStep");

  EXPECT_NE(spin, nullptr);
  EXPECT_NE(backup, nullptr);
  EXPECT_NE(lateral, nullptr);
}

TEST(PluginLoading, BtLibraryCanBeOpened)
{
  void * handle = dlopen("librobot_nav_recovery_bt_nodes.so", RTLD_LAZY);
  ASSERT_NE(handle, nullptr) << dlerror();
  EXPECT_NE(dlsym(handle, "BT_RegisterNodesFromPlugin"), nullptr) << dlerror();
  dlclose(handle);
}

TEST(PluginLoading, BtNodesRegisterWithFactory)
{
  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;
  factory.registerFromPlugin(loader.getOSName("robot_nav_recovery_bt_nodes"));

  const auto manifests = factory.manifests();
  EXPECT_NE(manifests.find("SelectRecoveryHeading"), manifests.end());
  EXPECT_NE(manifests.find("CheckSectorClear"), manifests.end());
  EXPECT_NE(manifests.find("MotionHealthy"), manifests.end());
  EXPECT_NE(manifests.find("EscalatingRecovery"), manifests.end());
  EXPECT_NE(manifests.find("GuardedLateralStep"), manifests.end());
}

}  // namespace robot_nav_recovery
