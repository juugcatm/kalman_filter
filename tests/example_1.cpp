#include "lib/kalman_filter.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace kalman {

  class NewtonianMotionTraits {
  public:
    static constexpr std::size_t N_states = 2;
    static constexpr std::size_t N_controls = 1;
    static constexpr std::size_t N_meas = 0;
  };

  template <typename T>
  class NewtonianFilter : public KalmanFilter<double, NewtonianMotionTraits> {
  public:
    void updateProcessMatrices(double dt) {
      states_T_states__process_(0, 1) = dt;
      states_T_controls__process_(0, 0) = 0.5 * pow(dt, 2);
      states_T_controls__process_(1, 0) = dt;
    }
  };
  
  class NewtonianMotionFixture
    : public ::testing::Test {
  public:
    using FilterType = NewtonianFilter<double>;
    using StatesVector = FilterType::StatesVector;
    using ControlsVector = FilterType::ControlsVector;
    FilterType filter_;
  };

  TEST_F(NewtonianMotionFixture, risingFluid) {
    StatesVector initial_state;
    initial_state[0] = 0;    // 0 m of position
    initial_state[1] = 1.0;  // 1 m/s velocity

    ControlsVector control_params = ControlsVector::Zero();
    filter_.initialize(initial_state, control_params);

    filter_.update(0.1);
    EXPECT_EQ(filter_.estimate()[0], 0.1);
    EXPECT_EQ(filter_.estimate()[1], 1.0);

    filter_.update(1.0);
    EXPECT_EQ(filter_.estimate()[0], 1.1);
    EXPECT_EQ(filter_.estimate()[1], 1.0);
  }
  
  TEST_F(NewtonianMotionFixture, fallingObject) {
    StatesVector initial_state;
    initial_state[0] = 0;    // 0 m of position
    initial_state[1] = 0;  // 1 m/s velocity

    ControlsVector controls = ControlsVector::Zero();
    controls[0] = -9.8;     // 1 g of gravity    
    filter_.initialize(initial_state, controls);

    filter_.update(0.1);
    EXPECT_DOUBLE_EQ(filter_.estimate()[0], -0.049);
    EXPECT_DOUBLE_EQ(filter_.estimate()[1], -0.98);

    filter_.update(1.0);
    EXPECT_DOUBLE_EQ(filter_.estimate()[0], -5.929);
    EXPECT_DOUBLE_EQ(filter_.estimate()[1], -10.78);
  }

  TEST_F(NewtonianMotionFixture, movingObject) {
    StatesVector initial_state;
    initial_state[0] = 0;    // 0 m of position
    initial_state[1] = 1.0;  // 1 m/s velocity

    ControlsVector control_params = ControlsVector::Zero();
    control_params[0] = 0.1;  // m/s^2 acceleration
    filter_.initialize(initial_state, control_params);

    filter_.update(0.1);
    EXPECT_EQ(filter_.estimate()[0], 0.1005);
    EXPECT_EQ(filter_.estimate()[1], 1.01);

    filter_.update(1.0);
    EXPECT_EQ(filter_.estimate()[0], 1.1605);
    EXPECT_EQ(filter_.estimate()[1], 1.11);
  }

}
