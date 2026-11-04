#include <chrono>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
    const auto up_topic = declare_parameter<std::string>(
      "up_topic", "/fastlio/up_body", startup_only);
    up_frame_ = declare_parameter<std::string>("up_frame", "body", startup_only);
    origin_frame_ = declare_parameter<std::string>("height_origin_frame", "BODY", startup_only);
    min_height_ = declare_parameter<double>("min_body_height", -.05, startup_only);
    max_height_ = declare_parameter<double>("max_body_height", 1., startup_only);
    max_gap_ = declare_parameter<double>("max_up_gap_s", .25, startup_only);
    wait_s_ = declare_parameter<double>("max_wait_s", .25, startup_only);
    max_age_ = declare_parameter<double>("max_cloud_age_s", .5, startup_only);
    if (up_topic.empty() || up_frame_.empty() || origin_frame_.empty() ||
      !std::isfinite(min_height_) || !std::isfinite(max_height_) || min_height_ > max_height_ ||
      !std::isfinite(max_gap_) || max_gap_ <= 0. || max_gap_ > 1. ||
      !std::isfinite(wait_s_) || wait_s_ <= 0. || wait_s_ > 1. ||
      !std::isfinite(max_age_) || max_age_ <= 0. || max_age_ > 2.)
    {
      throw std::invalid_argument("Invalid gravity height filter parameters");
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS().keep_last(1));
    up_subscription_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      up_topic, rclcpp::SensorDataQoS().keep_last(10),
      [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
        check_epoch();
        const auto stamp = stamp_ns(msg->header.stamp);
        const double age = (now().nanoseconds() - stamp) * 1e-9;
        if (msg->header.frame_id != up_frame_ || stamp <= 0 ||
          age > max_age_ || age < -.05 || !up_.add(stamp,
            Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z)))
        {
          ++up_drops_;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Rejecting gravity sample: frame=%s age_ms=%.1f (require %s, unit vector, increasing stamp); up_drops=%zu",
            msg->header.frame_id.c_str(), age * 1000., up_frame_.c_str(), up_drops_);
          return;
        }
        drain();
      });
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr input) {
        check_epoch();
        const auto stamp = stamp_ns(input->header.stamp);
        const double age = (now().nanoseconds() - stamp) * 1e-9;
        if (stamp <= last_cloud_stamp_ || stamp <= 0 || age > max_age_ || age < -.05 ||
          input->data.size() > 8 * 1024 * 1024)
        {
          ++cloud_drops_;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Rejecting stale/out-of-order/oversized cloud: age_ms=%.1f cloud_drops=%zu",
            age * 1000., cloud_drops_);
          return;
        }
        last_cloud_stamp_ = stamp;
        if (pending_.size() == 4) {pending_.pop_front(); ++cloud_drops_;}
        pending_.push_back({input, std::chrono::steady_clock::now()});
        drain();
      });
    // Nonblocking retry while waiting for timestamp brackets or static mount TF.
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {drain();});
    RCLCPP_INFO(get_logger(),
      "Costmap only: %s -> %s; %s +X +/-%.1f deg >=%.2f m, elsewhere >=%.2f m (sensor-origin XY range). Stamp/point records unchanged.",
      input_topic.c_str(), output_topic.c_str(), config.expected_frame_id.c_str(),
      config.front_half_angle_deg, config.front_min_range, config.other_min_range);
    RCLCPP_INFO(get_logger(),
      "Height: origin=%s, gravity from %s in %s, [%.3f, %.3f] m. "
      "Timestamp interpolation only; missing/stale direction or TF drops clouds. "
      "Marking and clearing use the same retained real endpoints.",
      origin_frame_.c_str(), up_topic.c_str(), up_frame_.c_str(), min_height_, max_height_);
  }

private:
  static int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {return 0;}
    return static_cast<int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
  }

  void check_epoch()
  {
    const auto clock_ns = now().nanoseconds();
    if (last_clock_ && clock_ns < last_clock_) {
      up_.clear(); pending_.clear(); last_cloud_stamp_ = 0;
    }
    last_clock_ = clock_ns;
  }

  Eigen::Isometry3d transform(const geometry_msgs::msg::Transform & t) const
  {
    Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
    if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.) > 1e-3 ||
      !result.translation().allFinite()) {throw std::invalid_argument("Invalid mount TF");}
    result.linear() = q.normalized().toRotationMatrix();
    return result;
  }

  void drain()
  {
    check_epoch();
    while (!pending_.empty()) {
      const auto & item = pending_.front();
      const auto & input = *item.cloud;
      const auto start = std::chrono::steady_clock::now();
      const auto ns = stamp_ns(input.header.stamp);
      const double wait = std::chrono::duration<double>(start - item.arrival).count();
      const double age = (now().nanoseconds() - ns) * 1e-9;
      if (wait > wait_s_ || age > max_age_ || age < -.05) {
        pending_.pop_front(); ++cloud_drops_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Height filter dropped cloud: no timely gravity/TF, or stale input. "
          "wait_ms=%.1f age_ms=%.1f cloud_drops=%zu up_drops=%zu",
          wait * 1000., age * 1000., cloud_drops_, up_drops_);
        continue;
      }
      const auto up = up_.at(ns, static_cast<int64_t>(max_gap_ * 1e9));
      if (!up) {break;}
      try {
        const rclcpp::Time stamp(input.header.stamp, get_clock()->get_clock_type());
        const auto sensor = tf_buffer_->lookupTransform(up_frame_, input.header.frame_id,
          stamp, rclcpp::Duration::from_seconds(0.));
        const auto body = tf_buffer_->lookupTransform(up_frame_, origin_frame_,
          stamp, rclcpp::Duration::from_seconds(0.));
        const auto plane = pongbot_navigation::make_height_plane(*up,
          transform(sensor.transform), transform(body.transform).translation(),
          min_height_, max_height_);
        auto result = filter_->filter(input, &plane);
        const double ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
        publisher_->publish(result.cloud);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "body_height_filter input=%zu kept=%zu range_drops=%zu height_drops=%zu "
          "nonfinite_drops=%zu filter_ms=%.3f wait_ms=%.1f age_ms=%.1f cloud_drops=%zu up_drops=%zu",
          result.input_points, result.kept_points, result.range_drops, result.height_drops,
          result.nonfinite_drops, ms, wait * 1000., age * 1000., cloud_drops_, up_drops_);
        pending_.pop_front();
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Height filter waiting for mount TF: %s", error.what());
        break;
      } catch (const std::invalid_argument & error) {
        pending_.pop_front(); ++cloud_drops_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Dropping invalid costmap cloud/height state: %s", error.what());
      }
    }
  }

  struct Pending
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    std::chrono::steady_clock::time_point arrival;
  };
  std::deque<Pending> pending_;
  pongbot_navigation::TimedUpBuffer up_;
  std::string up_frame_, origin_frame_;
  double min_height_, max_height_, max_gap_, wait_s_, max_age_;
  int64_t last_clock_{0}, last_cloud_stamp_{0};
  std::size_t cloud_drops_{0}, up_drops_{0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr up_subscription_;
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
