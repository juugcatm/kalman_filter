#include "lib/particle_motion_filter.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace kalman {

  TEST(ParticleMotionFilterTest, risingFluid) {
    ParticleMotionFilter<double> filter;
    filter.add_measurements(std::nullopt, MeasurementWithVariance<double>{1.0, 0}, std::nullopt);

    // Update for a small time step
    filter.update(0.1);
    EXPECT_EQ(filter.position(), 0.1);
    EXPECT_EQ(filter.velocity(), 1.0);
    EXPECT_EQ(filter.acceleration(), 0);
    
    // Update for larger time step
    EXPECT_EQ(filter.position(), 1.1);
    EXPECT_EQ(filter.velocity(), 1.0);
    EXPECT_EQ(filter.acceleration(), 0);
  }
  

}
