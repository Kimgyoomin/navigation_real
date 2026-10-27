#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include "pongbot_navigation/sector_filter.hpp"

class CostmapSectorFilter : public rclcpp::Node
{
public:
  CostmapSectorFilter() : Node("costmap_sector_filter")
  {
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    startup_only.description = "Startup-only setting; restart the filter to change it";
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/livox/lidar_pointcloud", startup_only);
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/livox/lidar_costmap", startup_only);
    if (input_topic.empty() || output_topic.empty() ||
      get_node_topics_interface()->resolve_topic_name(input_topic) ==
      get_node_topics_interface()->resolve_topic_name(output_topic))
    {
      throw std::invalid_argument("Filter input/output must be distinct nonempty topics");
    }
    pongbot_navigation::SectorConfig config;
    config.expected_frame_id = declare_parameter<std::string>(
      "expected_frame_id", config.expected_frame_id, startup_only);
    config.front_half_angle_deg = declare_parameter<double>(
      "front_half_angle_deg", config.front_half_angle_deg, startup_only);
    config.front_min_range = declare_parameter<double>(
      "front_min_range", config.front_min_range, startup_only);
    config.other_min_range = declare_parameter<double>(
      "other_min_range", config.other_min_range, startup_only);
    filter_ = std::make_unique<pongbot_navigation::SectorFilter>(config);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS().keep_last(1));
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr input) {filter_cloud(*input);});
    RCLCPP_INFO(get_logger(),
      "Costmap only: %s -> %s; %s +X +/-%.1f deg >=%.2f m, elsewhere >=%.2f m (sensor-origin XY range). Stamp/point records unchanged.",
      input_topic.c_str(), output_topic.c_str(), config.expected_frame_id.c_str(),
      config.front_half_angle_deg, config.front_min_range, config.other_min_range);
  }

private:
  void filter_cloud(const sensor_msgs::msg::PointCloud2 & input)
  {
    try {
      const auto start = std::chrono::steady_clock::now();
      auto result = filter_->filter(input);
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      publisher_->publish(result.cloud);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "sector_filter input=%zu kept=%zu range_drops=%zu nonfinite_drops=%zu filter_ms=%.3f",
        result.input_points, result.kept_points, result.range_drops,
        result.nonfinite_drops, elapsed_ms);
    } catch (const std::invalid_argument & error) {
      // Never bypass the mask, restamp, or republish an old cloud on failure.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Dropping invalid costmap cloud: %s", error.what());
    }
  }

  std::unique_ptr<pongbot_navigation::SectorFilter> filter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CostmapSectorFilter>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("costmap_sector_filter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
