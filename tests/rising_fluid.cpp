#include "lib/kalman_filter.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace kalman {

  class RisingFluidTraits {
  public:
    static constexpr std::size_t N_states = 2;
    static constexpr std::size_t N_controls = 0;
    static constexpr std::size_t N_meas = 0;
  };
  
  class KalmanFilterRisingFluidFixture
    : public ::testing::Test {
  public:
    using FilterType = KalmanFilter<double, RisingFluidTraits>;
    using StatesVector = FilterType::StatesVector;
    
    FilterType filter_;
  };
  
  TEST_F(KalmanFilterRisingFluidFixture, works) {
    StatesVector initial_state;
    initial_state[0] = 0;    // 0 m of position
    initial_state[1] = 1.0;  // 1 m/s velocity

    filter_.initialize(initial_state);

    filter_.update(0.1);
    EXPECT_EQ(filter_.estimate()[0], 0.1);
    EXPECT_EQ(filter_.estimate()[1], 1.0);

    filter_.update(1.0);
    EXPECT_EQ(filter_.estimate()[0], 1.1);
    EXPECT_EQ(filter_.estimate()[1], 1.0);
  }
 
}
