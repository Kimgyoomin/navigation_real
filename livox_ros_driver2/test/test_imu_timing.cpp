#include <gtest/gtest.h>
#include <atomic>
#include <thread>
#include "comm/lidar_imu_data_queue.h"

using livox_ros::imu_timing::State;

TEST(ImuTiming, SeparatesOldIngressFromQueueAndPublishDelay) {
  State state;
  state.window_start = 10.0;
  state.Receive(10.0, 100.0, 98.725, 1);
  state.Pop(10.002, 10.0);
  state.BeginPublish(10.003, 100.003, 98.725);
  state.EndPublish(10.004);
  auto s = state.Take(11.0, 0, livox_ros::imu_timing::Missing());
  EXPECT_NEAR(s.window.ingress_age_ms.Mean(), 1275.0, 1e-6);
  EXPECT_NEAR(s.window.queue_wait_ms.Max(), 2.0, 1e-6);
  EXPECT_NEAR(s.window.publish_work_ms.Max(), 1.0, 1e-6);
  EXPECT_EQ(s.window.published, 1u);
  EXPECT_TRUE(std::isnan(s.oldest_wait_ms));
}

TEST(ImuTiming, ReportsBlockedPublishWithoutWaitingForItToReturn) {
  State state;
  state.window_start = 0.0;
  state.BeginPublish(1.0, 100.0, 100.0);
  auto s = state.Take(2.0, 200, 1.005);
  EXPECT_EQ(s.window.published, 0u);
  EXPECT_NEAR(s.publish_inflight_ms, 1000.0, 1e-6);
  EXPECT_NEAR(s.oldest_wait_ms, 995.0, 1e-6);
  s = state.Take(3.0, 400, 1.005);
  EXPECT_NEAR(s.publish_inflight_ms, 2000.0, 1e-6);
  state.EndPublish(3.1);
  s = state.Take(4.0, 0, livox_ros::imu_timing::Missing());
  EXPECT_NEAR(s.window.publish_work_ms.Max(), 2100.0, 1e-6);
  EXPECT_TRUE(std::isnan(s.publish_inflight_ms));
}

TEST(ImuTiming, KeepsCrossWindowGapsAndNegativeClockAge) {
  State state;
  state.Receive(10.0, 100.0, 100.01, 1);
  auto s = state.Take(10.1, 1, 10.0);
  EXPECT_NEAR(s.window.ingress_age_ms.Max(), -10.0, 1e-6);
  state.Receive(11.28, 101.28, 101.28, 2);
  s = state.Take(12.0, 2, 10.0);
  EXPECT_NEAR(s.window.ingress_gap_ms.Max(), 1280.0, 1e-6);
  EXPECT_EQ(s.window.received, 1u);
}

TEST(ImuQueue, DiagnosticsPreserveFifoPayloadAndCountsUnderConcurrentSnapshots) {
  livox_ros::LidarImuDataQueue queue(true);
  constexpr unsigned count = 1000;
  std::atomic<bool> done{false};
  std::atomic<unsigned> errors{0};
  std::thread producer([&]() {
    for (unsigned i = 1; i <= count; ++i) {
      livox_ros::ImuData data{};
      data.time_stamp = i;
      data.handle = 145;
      data.gyro_x = static_cast<float>(i);
      data.acc_z = -1.0f;
      queue.Push(&data);
    }
  });
  std::thread consumer([&]() {
    for (unsigned i = 1; i <= count; ++i) {
      livox_ros::ImuData data{};
      while (!queue.Pop(data)) std::this_thread::yield();
      if (data.time_stamp != i || data.handle != 145 ||
          data.gyro_x != static_cast<float>(i) || data.acc_z != -1.0f) ++errors;
      queue.BeginPublish(data.time_stamp);
      queue.EndPublish();
    }
    done = true;
  });
  std::uint64_t received = 0, popped = 0, published = 0;
  auto sample = [&]() {
    livox_ros::imu_timing::Snapshot s;
    if (queue.TakeTimingSnapshot(s)) {
      received += s.window.received;
      popped += s.window.popped;
      published += s.window.published;
    }
  };
  while (!done) { sample(); std::this_thread::yield(); }
  producer.join();
  consumer.join();
  sample();
  EXPECT_EQ(errors, 0u);
  EXPECT_EQ(received, count);
  EXPECT_EQ(popped, count);
  EXPECT_EQ(published, count);
  EXPECT_TRUE(queue.Empty());
}

TEST(ImuQueue, DisabledDiagnosticsDoNotReportOrAlterSamples) {
  livox_ros::LidarImuDataQueue queue(false);
  livox_ros::ImuData data{};
  data.time_stamp = 12345;
  queue.Push(&data);
  EXPECT_TRUE(queue.Pop(data));
  EXPECT_EQ(data.time_stamp, 12345u);
  queue.BeginPublish(data.time_stamp);
  queue.EndPublish();
  livox_ros::imu_timing::Snapshot snapshot;
  EXPECT_FALSE(queue.TakeTimingSnapshot(snapshot));
}
