#pragma once

#include <cmath>
#include <stdexcept>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace pongbot_navigation
{
inline Eigen::Matrix3d velocity_skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d s;
  s << 0., -v.z(), v.y(), v.z(), 0., -v.x(), -v.y(), v.x(), 0.;
  return s;
}

// body_from_base describes the base origin/axes in the input child frame.
// Preserve acquisition time. Twist is shifted to the base origin AND rotated.
inline nav_msgs::msg::Odometry velocity_at_base(
  const nav_msgs::msg::Odometry & in,
  const geometry_msgs::msg::TransformStamped & body_from_base)
{
  const auto & t = body_from_base.transform.translation;
  const auto & q = body_from_base.transform.rotation;
  const auto & p = in.pose.pose;
  const auto & v = in.twist.twist.linear;
  const auto & w = in.twist.twist.angular;
  const Eigen::Vector3d arm(t.x, t.y, t.z), linear(v.x, v.y, v.z), angular(w.x, w.y, w.z);
  const Eigen::Vector3d position(p.position.x, p.position.y, p.position.z);
  Eigen::Quaterniond body_q_base(q.w, q.x, q.y, q.z);
  Eigen::Quaterniond world_q_body(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (in.child_frame_id != body_from_base.header.frame_id ||
      body_from_base.child_frame_id.empty() || in.header.frame_id.empty() ||
      !arm.allFinite() || !linear.allFinite() || !angular.allFinite() || !position.allFinite() ||
      !body_q_base.coeffs().allFinite() || !world_q_body.coeffs().allFinite() ||
      std::abs(body_q_base.norm()-1.) > 1e-3 || std::abs(world_q_body.norm()-1.) > 1e-3)
  {
    throw std::invalid_argument("Invalid velocity, pose, quaternion, or mount frames");
  }
  body_q_base.normalize();
  world_q_body.normalize();
  const Eigen::Matrix3d base_r_body = body_q_base.toRotationMatrix().transpose();
  const Eigen::Vector3d base_linear = base_r_body * (linear + angular.cross(arm));
  const Eigen::Vector3d base_angular = base_r_body * angular;
  const Eigen::Vector3d world_arm = world_q_body * arm;
  const Eigen::Vector3d base_position = position + world_arm;
  const Eigen::Quaterniond world_q_base = world_q_body * body_q_base;
  nav_msgs::msg::Odometry out = in;
  out.child_frame_id = body_from_base.child_frame_id;
  out.pose.pose.position.x = base_position.x();
  out.pose.pose.position.y = base_position.y();
  out.pose.pose.position.z = base_position.z();
  out.pose.pose.orientation.x = world_q_base.x();
  out.pose.pose.orientation.y = world_q_base.y();
  out.pose.pose.orientation.z = world_q_base.z();
  out.pose.pose.orientation.w = world_q_base.w();
  out.twist.twist.linear.x = base_linear.x();
  out.twist.twist.linear.y = base_linear.y();
  out.twist.twist.linear.z = base_linear.z();
  out.twist.twist.angular.x = base_angular.x();
  out.twist.twist.angular.y = base_angular.y();
  out.twist.twist.angular.z = base_angular.z();
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  Matrix6 pose_cov, twist_cov;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      pose_cov(i, j) = in.pose.covariance[6*i+j];
      twist_cov(i, j) = in.twist.covariance[6*i+j];
    }
  }
  if (!pose_cov.allFinite() || !twist_cov.allFinite()) {
    throw std::invalid_argument("Nonfinite input covariance");
  }
  Matrix6 twist_j = Matrix6::Zero();
  twist_j.block<3, 3>(0, 0) = base_r_body;
  twist_j.block<3, 3>(0, 3) = -base_r_body * velocity_skew(arm);
  twist_j.block<3, 3>(3, 3) = base_r_body;
  // ROS pose covariance uses fixed axes in header.frame_id. Mount treated as exact.
  Matrix6 pose_j = Matrix6::Identity();
  pose_j.block<3, 3>(0, 3) = -velocity_skew(world_arm);
  pose_cov = (pose_j * pose_cov * pose_j.transpose()).eval();
  twist_cov = (twist_j * twist_cov * twist_j.transpose()).eval();
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      out.pose.covariance[6*i+j] = pose_cov(i, j);
      out.twist.covariance[6*i+j] = twist_cov(i, j);
    }
  }
  return out;
}
}  // namespace pongbot_navigation
