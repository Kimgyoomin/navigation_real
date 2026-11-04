#include <gtest/gtest.h>
#include <limits>
#include "pongbot_navigation/height_filter.hpp"

using namespace pongbot_navigation;

TEST(HeightPlane, SensorOffsetAndInclusiveVerticalBounds)
{
  Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  sensor.translation() = Eigen::Vector3d(-.011, .02329, -.04412);
  const Eigen::Vector3d origin(-.1105, .02329, .09338);
  const auto plane = make_height_plane(Eigen::Vector3d::UnitZ(), sensor, origin, -.05, 1.);
  EXPECT_NEAR(plane.offset, -.1375, 1e-12);
  EXPECT_FALSE(plane.keep(2, 0, -.4));  // observed floor
  EXPECT_FALSE(plane.keep(2, 0, .08));  // below BODY-5 cm
  EXPECT_TRUE(plane.keep(2, 0, .1));
  EXPECT_TRUE(plane.keep(2, 0, 1.1));
  EXPECT_FALSE(plane.keep(2, 0, 1.2));
  HeightPlane boundary;
  EXPECT_TRUE(boundary.keep(0, 0, -.05));
  EXPECT_TRUE(boundary.keep(0, 0, 1.));
}

TEST(HeightPlane, PitchAndCommonWorldTranslationDoNotChangeRelativeHeight)
{
  const Eigen::Matrix3d R = (Eigen::AngleAxisd(.5, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(.21, Eigen::Vector3d::UnitY())).toRotationMatrix();
  Eigen::Isometry3d mount = Eigen::Isometry3d::Identity();
  mount.translation() = Eigen::Vector3d(.0995, 0, -.1375);
  const Eigen::Vector3d up = R.transpose() * Eigen::Vector3d::UnitZ();
  const auto plane = make_height_plane(up, mount, Eigen::Vector3d::Zero(), -.05, 1.);
  for (double drift : {0., .4, -2.}) {
    const Eigen::Vector3d base_world(10., 42., .7 + drift);
    for (double h : {-.5, -.08, 0., .5, 1.2}) {
      const Eigen::Vector3d p_world = base_world + Eigen::Vector3d(3, 0, h);
      const Eigen::Vector3d p_cloud = mount.inverse() * (R.transpose() * (p_world-base_world));
      EXPECT_NEAR(plane.normal.dot(p_cloud) + plane.offset, h, 1e-12);
      EXPECT_EQ(plane.keep(p_cloud.x(), p_cloud.y(), p_cloud.z()), h >= -.05 && h <= 1.);
    }
  }
}

TEST(TimedUp, RequiresAcquisitionTimeBracketsAndBoundedGap)
{
  TimedUpBuffer b;
  const int64_t t = 1000000000;
  const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  ASSERT_TRUE(b.add(t, up));
  EXPECT_FALSE(b.at(t-1, 250000000));
  EXPECT_FALSE(b.at(t+50000000, 250000000));
  const auto tilted = Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitY()) * up;
  ASSERT_TRUE(b.add(t+100000000, tilted));
  auto mid = b.at(t+50000000, 250000000);
  ASSERT_TRUE(mid);
  EXPECT_NEAR(mid->norm(), 1., 1e-12);
  EXPECT_NEAR(mid->x(), std::sin(.1), 1e-12);
  EXPECT_FALSE(b.at(t+50000000, 90000000));
  EXPECT_FALSE(b.at(t+100000001, 250000000));  // no latest-state fallback
  EXPECT_FALSE(b.add(t, up));
  EXPECT_FALSE(b.add(t+100000000, up));
  EXPECT_FALSE(b.add(t+200000000, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(b.add(t+200000000, Eigen::Vector3d(NAN,0,1)));
  b.clear();
  EXPECT_FALSE(b.at(t,250000000));
  EXPECT_TRUE(b.add(1,up));  // explicit clock reset accepts a new epoch
}

TEST(TimedUp, BoundsHistoryAndRejectsAntipodalInterpolation)
{
  TimedUpBuffer b;
  for (int i = 1; i <= 100; ++i) {ASSERT_TRUE(b.add(i, Eigen::Vector3d::UnitZ()));}
  EXPECT_FALSE(b.at(1, 100));
  EXPECT_TRUE(b.at(100, 100));
  ASSERT_TRUE(b.add(200, -Eigen::Vector3d::UnitZ()));
  EXPECT_FALSE(b.at(150, 100));
}

TEST(HeightPlane, InvalidPlaneIsRejected)
{
  HeightPlane h;
  h.normal.setZero(); EXPECT_THROW(h.validate(), std::invalid_argument);
  h.normal = Eigen::Vector3d::UnitZ();
  h.minimum = 2.; EXPECT_THROW(h.validate(), std::invalid_argument);
  h.minimum = -.05; h.offset = NAN; EXPECT_THROW(h.validate(), std::invalid_argument);
}
