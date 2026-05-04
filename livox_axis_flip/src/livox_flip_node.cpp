#include "livox_axis_flip/livox_flip_node.hpp"

#include <cstring>
#include <memory>
#include <utility>

namespace livox_axis_flip
{

LivoxFlipNode::LivoxFlipNode(const rclcpp::NodeOptions & options)
: Node("livox_flip_node", options)
{
  input_lidar_topic_ =
    this->declare_parameter<std::string>("input_lidar_topic", "/livox/lidar");
  input_imu_topic_ =
    this->declare_parameter<std::string>("input_imu_topic", "/livox/imu");
  output_lidar_topic_ =
    this->declare_parameter<std::string>("output_lidar_topic", "/livox/lidar_upright");
  output_imu_topic_ =
    this->declare_parameter<std::string>("output_imu_topic", "/livox/imu_upright");
  output_frame_id_ =
    this->declare_parameter<std::string>("output_frame_id", "livox_frame_upright");

  // 입력(raw sensor)은 sensor QoS
  auto input_qos = rclcpp::SensorDataQoS();

  // 출력(FAST-LIO feeding)은 Reliable
  rclcpp::QoS output_qos(rclcpp::KeepLast(10));
  output_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  output_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  lidar_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(output_lidar_topic_, output_qos);
  imu_pub_ =
    this->create_publisher<sensor_msgs::msg::Imu>(output_imu_topic_, output_qos);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_lidar_topic_, input_qos,
    std::bind(&LivoxFlipNode::lidar_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    input_imu_topic_, input_qos,
    std::bind(&LivoxFlipNode::imu_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LivoxFlipNode started");
  RCLCPP_INFO(this->get_logger(), " input_lidar_topic  : %s", input_lidar_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), " input_imu_topic    : %s", input_imu_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), " output_lidar_topic : %s", output_lidar_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), " output_imu_topic   : %s", output_imu_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), " output_frame_id    : %s", output_frame_id_.c_str());
}

std::optional<size_t> LivoxFlipNode::find_field_offset(
  const std::vector<sensor_msgs::msg::PointField> & fields,
  const std::string & name,
  uint8_t expected_datatype,
  uint32_t expected_count)
{
  for (const auto & f : fields) {
    if (f.name == name) {
      if (f.datatype != expected_datatype || f.count != expected_count) {
        return std::nullopt;
      }
      return static_cast<size_t>(f.offset);
    }
  }
  return std::nullopt;
}

bool LivoxFlipNode::init_xyz_offsets(const sensor_msgs::msg::PointCloud2 & msg)
{
  if (offsets_initialized_) {
    return true;
  }

  auto x_off = find_field_offset(msg.fields, "x");
  auto y_off = find_field_offset(msg.fields, "y");
  auto z_off = find_field_offset(msg.fields, "z");

  if (!x_off || !y_off || !z_off) {
    RCLCPP_ERROR(this->get_logger(), "Failed to find float32 x/y/z fields");
    return false;
  }

  xyz_offsets_.x = *x_off;
  xyz_offsets_.y = *y_off;
  xyz_offsets_.z = *z_off;
  xyz_offsets_.valid = true;
  offsets_initialized_ = true;

  RCLCPP_INFO(
    this->get_logger(),
    "XYZ offsets initialized: x=%zu y=%zu z=%zu point_step=%u",
    xyz_offsets_.x, xyz_offsets_.y, xyz_offsets_.z, msg.point_step);

  return true;
}

void LivoxFlipNode::lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!msg || msg->data.empty() || msg->width == 0 || msg->height == 0) {
    return;
  }

  if (!init_xyz_offsets(*msg)) {
    return;
  }

  const size_t num_points =
    static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  const size_t step = static_cast<size_t>(msg->point_step);
  const size_t total_bytes = msg->data.size();

  if (step == 0 || total_bytes < num_points * step) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Invalid PointCloud2 layout: step=%zu num_points=%zu data.size=%zu",
      step, num_points, total_bytes);
    return;
  }

  auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
  out->header = msg->header;
  out->header.frame_id = output_frame_id_;
  out->height = msg->height;
  out->width = msg->width;
  out->fields = msg->fields;
  out->is_bigendian = msg->is_bigendian;
  out->point_step = msg->point_step;
  out->row_step = msg->row_step;
  out->is_dense = msg->is_dense;

  // 현실적으로 필요한 최소 복사 1회
  out->data.resize(total_bytes);
  std::memcpy(out->data.data(), msg->data.data(), total_bytes);

  constexpr uint32_t SIGN_MASK = 0x80000000u;

  for (size_t i = 0; i < num_points; ++i) {
    auto * base = out->data.data() + i * step;

    auto * y_bits = reinterpret_cast<uint32_t *>(base + xyz_offsets_.y);
    auto * z_bits = reinterpret_cast<uint32_t *>(base + xyz_offsets_.z);

    *y_bits ^= SIGN_MASK;
    *z_bits ^= SIGN_MASK;
  }

  lidar_pub_->publish(std::move(out));
}

void LivoxFlipNode::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  sensor_msgs::msg::Imu out;
  out.header = msg->header;
  out.header.frame_id = output_frame_id_;

  out.orientation = msg->orientation;
  out.orientation_covariance = msg->orientation_covariance;

  // gyro: x 유지, y/z 반전
  out.angular_velocity.x = msg->angular_velocity.x;
  out.angular_velocity.y = -msg->angular_velocity.y;
  out.angular_velocity.z = -msg->angular_velocity.z;
  out.angular_velocity_covariance = msg->angular_velocity_covariance;

  // accel: raw 값 유지, y/z만 반전
  out.linear_acceleration.x = msg->linear_acceleration.x;
  out.linear_acceleration.y = -msg->linear_acceleration.y;
  out.linear_acceleration.z = -msg->linear_acceleration.z;
  out.linear_acceleration_covariance = msg->linear_acceleration_covariance;

  imu_pub_->publish(out);
}

}  // namespace livox_axis_flip

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<livox_axis_flip::LivoxFlipNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}