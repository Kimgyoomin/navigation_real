#include <memory>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

#include "nav2_core/global_planner.hpp"

TEST(HeightmapWavefrontPlugin, PluginlibDiscoversClass)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(
    "nav2_core", "nav2_core::GlobalPlanner");
  EXPECT_TRUE(loader.isClassAvailable("pongbot_navigation/HeightmapWavefrontNav2Planner"));
  auto planner = loader.createSharedInstance(
    "pongbot_navigation/HeightmapWavefrontNav2Planner");
  EXPECT_NE(planner, nullptr);
}
