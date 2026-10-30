#include <gtest/gtest.h>
#include "pipeline_timing.hpp"

TEST(PipelineTiming, PreservesStallAcrossReportBoundary)
{
    fastlio_timing::SensorWindow imu;
    imu.record(10.0, 100.001, 100.0, 100.0, 0.0);
    imu.reset_window();
    imu.record(11.28, 101.281, 100.005, 100.005, 0.2);
    EXPECT_EQ(imu.count, 1u);
    EXPECT_NEAR(imu.gap_ms.max(), 1280.0, 1e-8);
    EXPECT_NEAR(imu.stamp_step_ms.max(), 5.0, 1e-8);
    EXPECT_NEAR(imu.raw_age_ms.max(), 1276.0, 1e-8);
    // A burst of fresh messages must not hide the delayed message.
    imu.record(11.281, 101.282, 101.281, 101.281, 0.0);
    EXPECT_NEAR(imu.raw_age_ms.max(), 1276.0, 1e-8);
    EXPECT_NEAR(imu.raw_age_ms.mean(), 638.5, 1e-8);
}

TEST(PipelineTiming, DistinguishesTimestampOffsetFromDeliveryDelay)
{
    fastlio_timing::SensorWindow imu;
    imu.record(50.0, 200.001, 200.0, 199.9, 0.1);
    EXPECT_NEAR(imu.raw_age_ms.mean(), 1.0, 1e-8);
    EXPECT_NEAR(imu.corrected_age_ms.max(), 101.0, 1e-8);
    EXPECT_TRUE(std::isnan(imu.gap_ms.max()));
    imu.reset_window();
    EXPECT_EQ(imu.count, 0u);
    EXPECT_TRUE(std::isnan(imu.raw_age_ms.max()));
    EXPECT_DOUBLE_EQ(imu.last_entry, 50.0);
}

TEST(PipelineTiming, NegativeAgeIsNotClampedToZero)
{
    fastlio_timing::SensorWindow imu;
    imu.record(1.0, 10.0, 10.005, 10.005, 0.0);
    EXPECT_NEAR(imu.raw_age_ms.max(), -5.0, 1e-8);
}
