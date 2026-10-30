#ifndef LIVOX_ROS_DRIVER2_IMU_TIMING_H_
#define LIVOX_ROS_DRIVER2_IMU_TIMING_H_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>

namespace livox_ros {
namespace imu_timing {

inline bool Enabled() {
  static const bool enabled = std::getenv("LIVOX_DIAG_TIMING") != nullptr;
  return enabled;
}
inline double Missing() { return std::numeric_limits<double>::quiet_NaN(); }
inline double SteadyNow() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline double SystemNow() {
  return std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

struct Metric {
  std::uint64_t count = 0;
  double sum = 0.0;
  double peak = -std::numeric_limits<double>::infinity();
  void Add(double value) {
    if (!std::isfinite(value)) return;
    ++count;
    sum += value;
    peak = std::max(peak, value);
  }
  double Mean() const { return count ? sum / count : Missing(); }
  double Max() const { return count ? peak : Missing(); }
};

struct Window {
  std::uint64_t received = 0, popped = 0, published = 0;
  std::size_t queue_peak = 0;
  Metric ingress_gap_ms, stamp_step_ms, ingress_age_ms;
  Metric queue_wait_ms, publish_age_ms, publish_work_ms;
};

struct Snapshot {
  Window window;
  double seconds = 0.0;
  std::size_t queue_depth = 0;
  double oldest_wait_ms = Missing();
  double since_ingress_ms = Missing(), since_publish_ms = Missing();
  double publish_inflight_ms = Missing();
};

// The enclosing IMU queue mutex protects this constant-size accumulator.
// Acquisition age uses wall time; gaps and durations use a monotonic clock.
struct State {
  Window window;
  double window_start = SteadyNow();
  double last_ingress = Missing(), last_stamp = Missing();
  double last_publish = Missing(), publish_start = Missing();

  void Receive(double steady, double wall, double stamp, std::size_t depth) {
    ++window.received;
    window.ingress_gap_ms.Add((steady - last_ingress) * 1000.0);
    window.stamp_step_ms.Add((stamp - last_stamp) * 1000.0);
    window.ingress_age_ms.Add((wall - stamp) * 1000.0);
    window.queue_peak = std::max(window.queue_peak, depth);
    last_ingress = steady;
    last_stamp = stamp;
  }
  void Pop(double steady, double enqueued) {
    ++window.popped;
    window.queue_wait_ms.Add((steady - enqueued) * 1000.0);
  }
  void BeginPublish(double steady, double wall, double stamp) {
    publish_start = steady;
    window.publish_age_ms.Add((wall - stamp) * 1000.0);
  }
  void EndPublish(double steady) {
    ++window.published;
    window.publish_work_ms.Add((steady - publish_start) * 1000.0);
    last_publish = steady;
    publish_start = Missing();
  }
  Snapshot Take(double steady, std::size_t depth, double oldest_enqueued) {
    Snapshot result;
    result.window = window;
    result.seconds = steady - window_start;
    result.queue_depth = depth;
    result.oldest_wait_ms = (steady - oldest_enqueued) * 1000.0;
    result.since_ingress_ms = (steady - last_ingress) * 1000.0;
    result.since_publish_ms = (steady - last_publish) * 1000.0;
    result.publish_inflight_ms = (steady - publish_start) * 1000.0;
    window = {};
    window.queue_peak = depth;
    window_start = steady;
    // Retain last arrivals and an in-flight publish across reporting windows.
    return result;
  }
};

}  // namespace imu_timing
}  // namespace livox_ros
#endif
