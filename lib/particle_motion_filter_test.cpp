#include "lib/particle_motion_filter.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace kalman {

  TEST(ParticleMotionFilterTest, risingFluid) {
    ParticleMotionFilter<double> filter;
    EXPECT_EQ(filter.position(), 0);
    EXPECT_EQ(filter.velocity(), 0);
    EXPECT_EQ(filter.acceleration(), 0);

    // Initialize the filter with the provided measurement
    filter.add_measurements(std::nullopt, MeasurementWithVariance<double>{1.0, 0.1}, std::nullopt);

    // Update for a small time step
    filter.update(0.1);
    EXPECT_EQ(filter.position(), 0.1);
    EXPECT_EQ(filter.velocity(), 1.0);
    EXPECT_EQ(filter.acceleration(), 0);
    
    // Update for larger time step
    filter.update(1.0);
    EXPECT_EQ(filter.position(), 1.1);
    EXPECT_EQ(filter.velocity(), 1.0);
    EXPECT_EQ(filter.acceleration(), 0);
  }
  

}
