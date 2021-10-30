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
    FilterType filter_;
  };

  TEST_F(AirplaneTrackingFixture, trackAirplane) {
    StatesVector initial_state;
    initial_state[0] = 4000; // m of position
    initial_state[1] = 280;  // m/s velocity

    ControlsVector control_params;
    control_params[0] = 2;  // m/s^2 acceleration

    // Initialize the filter with the initial state.
    filter_.initialize(initial_state, control_params);

    // Define the list of measurements
    std::vector<MeasurementVector> measurements;
    measurements.emplace_back(4260, 282);
    measurements.emplace_back(4550, 285);
    measurements.emplace_back(4860, 286);
    measurements.emplace_back(5110, 290);

    // Define the measurement covariance.
    MeasurementVector measurement_cov_diag;
    measurement_cov_diag[0] = 25; // m of position
    measurement_cov_diag[1] = 6;  // m/s of velocity
    auto measurement_cov = measurement_cov_diag.asDiagonal();
    auto measurement_T_states = Eigen::Matrix<double, 2, 2>::Identity();
    
    for (std::size_t i = 0; i < measurements.size(); i++) {
      filter_.update(1);  // s of elapsed time
      filter_.add_measurement(measurements[i], measurement_cov, measurement_T_states);
    }
    
  }

}
