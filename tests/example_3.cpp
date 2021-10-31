#include "lib/kalman_filter.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace kalman {

  /** PolyFit Example:
   * Try to fit a 2nd degree polynomial to incoming data.
   * Estimates the three polynomial coefficients.
   */
  class PolyFitTraits {
  public:
    static constexpr std::size_t N_states = 3;
    static constexpr std::size_t N_controls = 1;  // TODO make it work for 0 controls
    static constexpr std::size_t N_meas = 1;
  };

  template <typename T>
  class PolyFitFilter : public KalmanFilter<double, PolyFitTraits> {
  public:
    void updateProcessMatrices(double dt) {
      states_T_states__process_(0, 1) = dt;
      states_T_states__process_(0, 2) = 0.5 * pow(dt, 2);
      states_T_states__process_(1, 2) = dt;
    }
  };
  
  class PolyFitFixture
    : public ::testing::Test {
  public:
    using FilterType = PolyFitFilter<double>;
    using StatesVector = FilterType::StatesVector;
    using ControlsVector = FilterType::ControlsVector;
    using MeasurementVector = FilterType::MeasurementVector;
    using MeasurementMatrix = FilterType::MeasurementMatrix;
    using ProcessMatrix = FilterType::ProcessMatrix;
    FilterType filter_;
  };

  TEST_F(PolyFitFixture, trackPolynomial) {
    // Define the list of measurements
    std::vector<MeasurementVector> measurements;
    measurements.emplace_back(0.15);
    measurements.emplace_back(0.17);
    measurements.emplace_back(0.169);
    measurements.emplace_back(0.17);
    measurements.emplace_back(0.17);
    measurements.emplace_back(0.14);
    measurements.emplace_back(0.16);
    measurements.emplace_back(0.16);
    measurements.emplace_back(0.19);
    measurements.emplace_back(0.20);
    measurements.emplace_back(0.21);

    // Define the measurement covariance.
    MeasurementVector measurement_cov;
    measurement_cov[0] = pow(0.01, 2); // Accurate to 2 decimal places
    Eigen::Matrix<double, 1, 3> measurement_T_states = Eigen::Matrix<double, 1, 3>::Zero();
    measurement_T_states[0] = 1;

    StatesVector initial_state;
    initial_state[0] = measurements[0][0];
    initial_state[1] = 0;
    initial_state[2] = 0;
    
    // TODO: Allow for 0 size control matrix
    // For now we just for a zero here.
    ControlsVector control_params;
    control_params[0] = 0;  // Unused

    StatesVector process_cov;
    process_cov[0] = pow(0.01, 2); // Accruate to 2 decimal places
    process_cov[1] = 0; // No uncertainty here
    process_cov[2] = 0; // No uncertainty here
    
    // Initialize the filter with the initial state.
    filter_.initialize(initial_state, control_params, process_cov);
    
    for (std::size_t i = 1; i < measurements.size(); i++) {
      filter_.update(1);  // units of elapsed time
      filter_.add_measurement(measurements[i], measurement_cov, measurement_T_states);
    }
  }

}
