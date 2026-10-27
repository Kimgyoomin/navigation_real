#include <gtest/gtest.h>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Geometry>
#include "velocity_output.hpp"


TEST(VelocityOutput, PreservesAcquisitionStampFramesAndLegacyMessage)
{
  nav_msgs::msg::Odometry source, output;
  source.header.stamp.sec = 1234;
  source.header.stamp.nanosec = 567890123;
  source.header.frame_id = "camera_init";
  source.child_frame_id = "body";
  source.pose.pose.position.x = 4.;
  source.pose.pose.orientation.w = 1.;
  source.pose.covariance[7] = .25;
  fastlio_output::BodyVelocity velocity;
  velocity.linear = {1., 2., 3.};
  velocity.angular = {.1, .2, .3};
  velocity.covariance = Eigen::Matrix<double, 6, 6>::Identity() * .01;
  velocity.covariance(0, 4) = velocity.covariance(4, 0) = .002;
  ASSERT_TRUE(fastlio_output::copy_with_body_velocity(source, velocity, output));
  EXPECT_EQ(output.header, source.header);
  EXPECT_EQ(output.child_frame_id, source.child_frame_id);
  EXPECT_EQ(output.pose, source.pose);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.x, 1.);
  EXPECT_DOUBLE_EQ(output.twist.twist.angular.z, .3);
  EXPECT_DOUBLE_EQ(output.twist.covariance[4], .002);
  EXPECT_DOUBLE_EQ(output.twist.covariance[24], .002);
  EXPECT_DOUBLE_EQ(source.twist.twist.linear.x, 0.);
  EXPECT_DOUBLE_EQ(source.twist.twist.angular.z, 0.);
  EXPECT_DOUBLE_EQ(source.twist.covariance[4], 0.);
}

TEST(VelocityOutput, RejectsNonFiniteEstimateWithoutChangingOutput)
{
  nav_msgs::msg::Odometry source, output;
  output.child_frame_id = "unchanged";
  fastlio_output::BodyVelocity velocity;
  velocity.linear.setZero();
  velocity.angular.setZero();
  velocity.covariance.setIdentity();
  velocity.linear.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(fastlio_output::copy_with_body_velocity(source, velocity, output));
  velocity.linear.setZero();
  velocity.angular.z() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(fastlio_output::copy_with_body_velocity(source, velocity, output));
  velocity.angular.setZero();
  velocity.covariance(5, 5) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(fastlio_output::copy_with_body_velocity(source, velocity, output));
  EXPECT_EQ(output.child_frame_id, "unchanged");
}

TEST(VelocityOutput, RotatesWorldVelocityAndSubtractsPosteriorBias)
{
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(
      1.5707963267948966, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Matrix<double, 23, 23> p = Eigen::Matrix<double, 23, 23>::Zero();
  p(12, 12) = .01;
  p(13, 13) = .04;
  p(17, 17) = .02;
  const auto out = fastlio_output::body_velocity(rotation, {0., 1., 0.},
      {0., 0., .6}, {0., 0., .1}, p, {.001, .001, .001});
  EXPECT_NEAR(out.linear.x(), 1., 1e-12);
  EXPECT_NEAR(out.linear.y(), 0., 1e-12);
  EXPECT_NEAR(out.angular.z(), .5, 1e-12);
  EXPECT_NEAR(out.covariance(0, 0), .04, 1e-12);
  EXPECT_NEAR(out.covariance(1, 1), .01, 1e-12);
  EXPECT_NEAR(out.covariance(5, 5), .021, 1e-12);
}

TEST(VelocityOutput, OrientationUncertaintyContributesToTwist)
{
  Eigen::Matrix<double, 23, 23> p = Eigen::Matrix<double, 23, 23>::Zero();
  p(5, 5) = .03;
  const auto out = fastlio_output::body_velocity(Eigen::Matrix3d::Identity(),
      {2., 0., 0.}, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), p,
      Eigen::Vector3d::Zero());
  EXPECT_NEAR(out.covariance(1, 1), .12, 1e-12);
  EXPECT_TRUE(out.covariance.isApprox(out.covariance.transpose()));
}

TEST(VelocityOutput, StationaryAndRotatedMount)
{
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(
      3.141592653589793, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Matrix<double, 23, 23> p =
      Eigen::Matrix<double, 23, 23>::Identity() * .01;
  const auto stopped = fastlio_output::body_velocity(rotation,
      Eigen::Vector3d::Zero(), {.01, -.02, .03}, {.01, -.02, .03},
      p, Eigen::Vector3d::Constant(.001));
  EXPECT_TRUE(stopped.linear.isZero());
  EXPECT_TRUE(stopped.angular.isZero());
  const auto moving = fastlio_output::body_velocity(rotation, {.2, .3, .4},
      {0., 0., -.5}, Eigen::Vector3d::Zero(), p, Eigen::Vector3d::Zero());
  EXPECT_TRUE(moving.linear.isApprox(Eigen::Vector3d(.2, -.3, -.4)));
  EXPECT_NEAR(moving.angular.z(), -.5, 1e-12);
  EXPECT_TRUE(moving.covariance.allFinite());
  EXPECT_TRUE(moving.covariance.isApprox(moving.covariance.transpose()));
}
