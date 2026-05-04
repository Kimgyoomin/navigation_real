#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <optional>
#include <string>
#include <vector>

namespace livox_axis_flip
{

class LivoxFlipNode : public rclcpp::Node
{
public:
  explicit LivoxFlipNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct XYZOffsets
  {
    size_t x{0};
    size_t y{0};
    size_t z{0};
    bool valid{false};
  };

  static std::optional<size_t> find_field_offset(
    const std::vector<sensor_msgs::msg::PointField> & fields,
    const std::string & name,
    uint8_t expected_datatype = sensor_msgs::msg::PointField::FLOAT32,
    uint32_t expected_count = 1);

  bool init_xyz_offsets(const sensor_msgs::msg::PointCloud2 & msg);

  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

private:
  std::string input_lidar_topic_;
  std::string input_imu_topic_;
  std::string output_lidar_topic_;
  std::string output_imu_topic_;
  std::string output_frame_id_;

  bool offsets_initialized_{false};
  XYZOffsets xyz_offsets_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

}  // namespace livox_axis_flip