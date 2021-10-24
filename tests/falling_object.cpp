#include "lib/kalman_filter.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace kalman {

  class FallingObjectTraits {
  public:
    static constexpr std::size_t N_states = 2;
    static constexpr std::size_t N_controls = 1;
    static constexpr std::size_t N_meas = 0;
  };
  
  class KalmanFilterFallingObjectFixture
    : public ::testing::Test {
  public:
    using FilterType = KalmanFilter<double, FallingObjectTraits>;
    using StatesVector = FilterType::StatesVector;
    using ControlsVector = FilterType::ControlsVector;
    
    FilterType filter_;
  };
  
  TEST_F(KalmanFilterFallingObjectFixture, works) {
    StatesVector initial_state;
    initial_state[0] = 0;    // 0 m of position
    initial_state[1] = 0;  // 1 m/s velocity

    ControlsVector controls;
    controls[0] = -9.8;     // 1 g of gravity
    
    filter_.initialize(initial_state, controls);

    filter_.update(0.1);
    EXPECT_DOUBLE_EQ(filter_.estimate()[0], -0.049);
    EXPECT_DOUBLE_EQ(filter_.estimate()[1], -0.98);

    filter_.update(1.0);
    EXPECT_DOUBLE_EQ(filter_.estimate()[0], -5.929);
    EXPECT_DOUBLE_EQ(filter_.estimate()[1], -10.78);
  }
 
}
