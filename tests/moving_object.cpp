#include "lib/kalman_filter.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace kalman {

  class MovingObjectTraits {
  public:
    static constexpr std::size_t N_states = 2;
    static constexpr std::size_t N_controls = 1;
    static constexpr std::size_t N_meas = 0;
  };
  
  class KalmanFilterMovingObjectFixture
    : public ::testing::Test {
  public:
    using FilterType = KalmanFilter<double, MovingObjectTraits>;
    using StatesVector = FilterType::StatesVector;
    using ControlsVector = FilterType::ControlsVector;
    FilterType filter_;
  };
  
  TEST_F(KalmanFilterMovingObjectFixture, works) {
    StatesVector initial_state;
    initial_state[0] = 0;    // 0 m of position
    initial_state[1] = 1.0;  // 1 m/s velocity

    ControlsVector control_params = ControlsVector::Zero();
    control_params[0] = 0.1;  // m/s^2 acceleration
    filter_.initialize(initial_state, control_params);

    filter_.update(0.1);
    LOG(INFO) << filter_.estimate()[0];
    EXPECT_EQ(filter_.estimate()[0], 0.1005);
    EXPECT_EQ(filter_.estimate()[1], 1.01);

    filter_.update(1.0);
    EXPECT_EQ(filter_.estimate()[0], 1.1605);
    EXPECT_EQ(filter_.estimate()[1], 1.11);
  }
 
}
