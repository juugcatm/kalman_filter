#include "lib/kalman_filter.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace kalman {

  class AirplaneTrackingTraits {
  public:
    static constexpr std::size_t N_states = 2;
    static constexpr std::size_t N_controls = 1;
    static constexpr std::size_t N_meas = 2;
  };

  template <typename T>
  class AirplaneTrackingFilter : public KalmanFilter<double, AirplaneTrackingTraits> {
  public:
    void updateProcessMatrices(double dt) {
      states_T_states__process_(0, 1) = dt;
      states_T_controls__process_(0, 0) = 0.5 * pow(dt, 2);
      states_T_controls__process_(1, 0) = dt;
    }
  };
  
  class AirplaneTrackingFixture
    : public ::testing::Test {
  public:
    using FilterType = AirplaneTrackingFilter<double>;
    using StatesVector = FilterType::StatesVector;
    using ControlsVector = FilterType::ControlsVector;
    using MeasurementVector = FilterType::MeasurementVector;
    using MeasurementMatrix = FilterType::MeasurementMatrix;
    using ProcessMatrix = FilterType::ProcessMatrix;
    FilterType filter_;
  };

  TEST_F(AirplaneTrackingFixture, trackAirplane) {
    StatesVector initial_state;
    initial_state[0] = 4000; // m of position
    initial_state[1] = 280;  // m/s velocity

    ControlsVector control_params;
    control_params[0] = 2;  // m/s^2 acceleration

    StatesVector process_cov;
    process_cov[0] = pow(20, 2); // 20 m of intial position cov
    process_cov[1] = pow(5, 2);  // 5 m/s of initial velocity cov
    
    // Initialize the filter with the initial state.
    filter_.initialize(initial_state, control_params, process_cov);

    // Define the list of measurements
    std::vector<MeasurementVector> measurements;
    measurements.emplace_back(4260, 282);
    measurements.emplace_back(4550, 285);

    // Define the measurement covariance.
    MeasurementVector measurement_cov;
    measurement_cov[0] = pow(25, 2); // 25 m of position accuracy
    measurement_cov[1] = pow(6, 2);  // 6 m/s of velocity accuracy
    auto measurement_T_states = Eigen::Matrix<double, 2, 2>::Identity();

    const std::vector<StatesVector> exp_est = {
      {4272.5, 282},
      {4553.8505470722612, 284.2906976744186}
    };

    const std::vector<StatesVector> exp_cov = {
      {252.97619047619048, 14.754098360655737},
      {187.43783269754172, 10.465116279069768}
    };
    
    for (std::size_t i = 0; i < measurements.size(); i++) {
      filter_.update(1);  // s of elapsed time
      filter_.add_measurement(measurements[i], measurement_cov, measurement_T_states);

      const auto& est = filter_.estimate();
      EXPECT_DOUBLE_EQ(est[0], exp_est[i][0]);
      EXPECT_DOUBLE_EQ(est[1], exp_est[i][1]);

      const auto& cov = filter_.covariance();
      EXPECT_DOUBLE_EQ(cov[0], exp_cov[i][0]);
      EXPECT_DOUBLE_EQ(cov[1], exp_cov[i][1]);
    }
    
  }

}
