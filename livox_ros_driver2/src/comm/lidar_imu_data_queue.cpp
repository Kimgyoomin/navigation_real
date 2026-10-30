//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lidar_imu_data_queue.h"

namespace livox_ros {

void LidarImuDataQueue::Push(ImuData* imu_data) {
  ImuData data{};
  data.lidar_type = imu_data->lidar_type;
  data.handle = imu_data->handle;
  data.time_stamp = imu_data->time_stamp;

  data.gyro_x = imu_data->gyro_x;
  data.gyro_y = imu_data->gyro_y;
  data.gyro_z = imu_data->gyro_z;

  data.acc_x = imu_data->acc_x;
  data.acc_y = imu_data->acc_y;
  data.acc_z = imu_data->acc_z;

  std::lock_guard<std::mutex> lock(mutex_);
  Entry entry;
  entry.data = data;
  entry.enqueued = diagnostics_ ? imu_timing::SteadyNow() : 0.0;
  imu_data_queue_.push_back(entry);
  if (diagnostics_) {
    timing_.Receive(entry.enqueued, imu_timing::SystemNow(),
                    static_cast<double>(data.time_stamp) * 1.0e-9,
                    imu_data_queue_.size());
  }
}

bool LidarImuDataQueue::Pop(ImuData& imu_data) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (imu_data_queue_.empty()) {
    return false;
  }
  imu_data = imu_data_queue_.front().data;
  if (diagnostics_) {
    timing_.Pop(imu_timing::SteadyNow(), imu_data_queue_.front().enqueued);
  }
  imu_data_queue_.pop_front();
  return true;
}

bool LidarImuDataQueue::Empty() {
  std::lock_guard<std::mutex> lock(mutex_);
  return imu_data_queue_.empty();
}

void LidarImuDataQueue::Clear() {
  std::list<Entry> tmp_imu_data_queue;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_data_queue_.swap(tmp_imu_data_queue);
  }
}

void LidarImuDataQueue::BeginPublish(std::uint64_t stamp) {
  if (!diagnostics_) return;
  std::lock_guard<std::mutex> lock(mutex_);
  timing_.BeginPublish(imu_timing::SteadyNow(), imu_timing::SystemNow(),
                       static_cast<double>(stamp) * 1.0e-9);
}

void LidarImuDataQueue::EndPublish() {
  if (!diagnostics_) return;
  std::lock_guard<std::mutex> lock(mutex_);
  timing_.EndPublish(imu_timing::SteadyNow());
}

bool LidarImuDataQueue::TakeTimingSnapshot(imu_timing::Snapshot& snapshot) {
  if (!diagnostics_) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(timing_.last_ingress)) {
    // Do not include the driver's startup wait in the first input-rate window.
    timing_.window_start = imu_timing::SteadyNow();
    return false;
  }
  snapshot = timing_.Take(imu_timing::SteadyNow(), imu_data_queue_.size(),
      imu_data_queue_.empty() ? imu_timing::Missing() : imu_data_queue_.front().enqueued);
  return true;
}

} // namespace livox_ros
