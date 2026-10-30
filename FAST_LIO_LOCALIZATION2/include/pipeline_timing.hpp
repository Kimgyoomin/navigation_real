#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

// Bounded diagnostic accumulators only: no message storage or filtering.
// Callers provide synchronization when a sensor stream crosses callback groups.
namespace fastlio_timing
{
inline double missing() { return std::numeric_limits<double>::quiet_NaN(); }
inline double steady_seconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Metric
{
    std::uint64_t count = 0;
    double sum = 0.0;
    double peak = -std::numeric_limits<double>::infinity();
    void add(double value)
    {
        if (!std::isfinite(value)) return;
        ++count;
        sum += value;
        peak = std::max(peak, value);
    }
    double mean() const { return count ? sum / count : missing(); }
    double max() const { return count ? peak : missing(); }
};

struct SensorWindow
{
    std::uint64_t count = 0;
    double last_entry = missing();
    double last_stamp = missing();
    Metric gap_ms, stamp_step_ms, raw_age_ms, corrected_age_ms, lock_ms;

    void record(double entry, double now_ros, double raw_stamp,
                double corrected_stamp, double lock_wait_ms)
    {
        ++count;
        gap_ms.add((entry - last_entry) * 1000.0);
        stamp_step_ms.add((raw_stamp - last_stamp) * 1000.0);
        raw_age_ms.add((now_ros - raw_stamp) * 1000.0);
        corrected_age_ms.add((now_ros - corrected_stamp) * 1000.0);
        lock_ms.add(lock_wait_ms);
        last_entry = entry;
        last_stamp = raw_stamp;
    }

    void reset_window()
    {
        // Keep the last arrival across reports so a 1.3 s stall cannot vanish
        // just because it straddles two reporting windows.
        count = 0;
        gap_ms = {}; stamp_step_ms = {}; raw_age_ms = {};
        corrected_age_ms = {}; lock_ms = {};
    }
};

class ScopedDuration
{
public:
    explicit ScopedDuration(Metric *metric)
        : metric_(metric), start_(metric ? steady_seconds() : 0.0) {}
    ~ScopedDuration()
    {
        if (metric_) metric_->add((steady_seconds() - start_) * 1000.0);
    }
    ScopedDuration(const ScopedDuration &) = delete;
    ScopedDuration &operator=(const ScopedDuration &) = delete;
private:
    Metric *metric_;
    double start_;
};
}  // namespace fastlio_timing
