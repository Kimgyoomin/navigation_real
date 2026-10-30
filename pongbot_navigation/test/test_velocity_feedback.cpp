#include <limits>
#include <gtest/gtest.h>
#include "pongbot_navigation/velocity_feedback.hpp"

namespace
{
nav_msgs::msg::Odometry input()
{
  nav_msgs::msg::Odometry m;
  m.header.frame_id = "camera_init";
  m.header.stamp.sec = 42;
  m.header.stamp.nanosec = 123456789;
  m.child_frame_id = "body";
  m.pose.pose.orientation.w = 1.;
  for (int i = 0; i < 6; ++i) {
    m.pose.covariance[7*i] = .01;
    m.twist.covariance[7*i] = .01;
  }
  return m;
}
geometry_msgs::msg::TransformStamped mount()
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "body";
  tf.child_frame_id = "base_link";
  tf.transform.rotation.w = 1.;
  return tf;
}
}

TEST(VelocityFeedback, ActualMountHasCorrectCrossProductSignAndStamp)
{
  auto m = input(); auto tf = mount();
  tf.transform.translation.x = -.1105;
  tf.transform.translation.y = .02329;
  tf.transform.translation.z = .09338;
  m.twist.twist.linear.x = .5;
  m.twist.twist.angular.z = 1.;
  const auto out = pongbot_navigation::velocity_at_base(m, tf);
  EXPECT_EQ(out.header, m.header);
  EXPECT_EQ(out.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(out.twist.twist.linear.x, .5-.02329);
  EXPECT_DOUBLE_EQ(out.twist.twist.linear.y, -.1105);
  EXPECT_DOUBLE_EQ(out.twist.twist.angular.z, 1.);
  EXPECT_DOUBLE_EQ(out.pose.pose.position.x, -.1105);
  EXPECT_DOUBLE_EQ(m.twist.twist.linear.y, 0.);
}

TEST(VelocityFeedback, RotatesBothTwistAndPoseForNonAlignedMount)
{
  auto m = input(); auto tf = mount();
  // Base X is body Y, and body X is world Y.
  const auto q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));
  tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
  m.pose.pose.orientation.z = q.z(); m.pose.pose.orientation.w = q.w();
  tf.transform.translation.x = 1.;
  m.twist.twist.linear.y = 2.;
  m.twist.twist.angular.z = 1.;
  const auto out = pongbot_navigation::velocity_at_base(m, tf);
  EXPECT_NEAR(out.twist.twist.linear.x, 3., 1e-12);
  EXPECT_NEAR(out.twist.twist.linear.y, 0., 1e-12);
  EXPECT_NEAR(out.pose.pose.position.y, 1., 1e-12);
  EXPECT_NEAR(std::abs(out.pose.pose.orientation.z), 1., 1e-12);
}

TEST(VelocityFeedback, CovarianceIncludesLeverArmAndIsPositiveSemidefinite)
{
  auto m = input(); auto tf = mount(); tf.transform.translation.x = 2.;
  const auto out = pongbot_navigation::velocity_at_base(m, tf);
  EXPECT_NEAR(out.twist.covariance[7], .05, 1e-12);
  EXPECT_NEAR(out.twist.covariance[11], .02, 1e-12);
  EXPECT_NEAR(out.twist.covariance[31], .02, 1e-12);
  EXPECT_NEAR(out.pose.covariance[7], .05, 1e-12);
  // J C J^T is PSD; check several arbitrary quadratic forms.
  Eigen::Matrix<double, 6, 6> c;
  for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) c(i,j)=out.twist.covariance[6*i+j];
  EXPECT_TRUE(c.isApprox(c.transpose()));
  for (int i = 0; i < 30; ++i) {
    const Eigen::Matrix<double, 6, 1> x = Eigen::Matrix<double, 6, 1>::Random();
    EXPECT_GE((x.transpose()*c*x)(0), 0.);
  }
}

TEST(VelocityFeedback, RejectsWrongFramesInvalidQuaternionAndNonfiniteInput)
{
  auto m = input(); auto tf = mount();
  tf.header.frame_id = "other";
  EXPECT_THROW(pongbot_navigation::velocity_at_base(m, tf), std::invalid_argument);
  tf = mount(); m.pose.pose.orientation.w = 0.;
  EXPECT_THROW(pongbot_navigation::velocity_at_base(m, tf), std::invalid_argument);
  m = input(); m.twist.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(pongbot_navigation::velocity_at_base(m, tf), std::invalid_argument);
  m = input(); m.twist.covariance[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(pongbot_navigation::velocity_at_base(m, tf), std::invalid_argument);
}
