#include "lib/kalman_filter.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace kalman {

  template <typename T>
  class NewtonianFilter : public KalmanFilter<double, 2, 1, 2> {
  public:
    Eigen::Matrix<T, 2, 2> A (dt) {
      auto A = Eigen::Matrix<T, 2, 2>::Identity();
      A(0,1) = dt;
      return A;
    }

    Eigen::Matrix<T, 2, 1> B (dt) {
      auto B = Eigen::Matrix<T, 2, 1>::Zero();
      B[0] = 0.5 * pow(dt, 2);
      B[1] = dt;
      return B;
    }
    
    Eigen::Matrix<T, 2, 2> H () {
      return Eigen::Matrix<T, 2, 2>::Identity();
    }
  };
  
  class NewtonianMotionFixture
    : public ::testing::Test {
  public:
    using FilterType = NewtonianFilter<double>;
    FilterType filter_;
  };

  TEST_F(NewtonianMotionFixture, risingFluid) {
    StatesVector initial_state;
    initial_state[0] = 0;    // 0 m of position
    initial_state[1] = 1.0;  // 1 m/s velocity

    ControlsVector control_params = ControlsVector::Zero();
    StatesVector process_cov = StatesVector::Zero();
    filter_.initialize(initial_state, control_params, process_cov);

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
    StatesVector process_cov = StatesVector::Zero();
    controls[0] = -9.8;     // 1 g of gravity    
    filter_.initialize(initial_state, controls, process_cov);

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
    StatesVector process_cov = StatesVector::Zero();
    control_params[0] = 0.1;  // m/s^2 acceleration
    filter_.initialize(initial_state, control_params, process_cov);

    filter_.update(0.1);
    EXPECT_EQ(filter_.estimate()[0], 0.1005);
    EXPECT_EQ(filter_.estimate()[1], 1.01);

    filter_.update(1.0);
    EXPECT_EQ(filter_.estimate()[0], 1.1605);
    EXPECT_EQ(filter_.estimate()[1], 1.11);
  }

}
