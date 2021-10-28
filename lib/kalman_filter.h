#pragma once

#include <utility>

#include <Eigen/Dense>

namespace kalman {

  template <typename T, typename Traits>
  class KalmanFilter {
  public:
    using StatesVector = Eigen::Matrix<T, Traits::N_states, 1>;
    using ProcessMatrix = Eigen::Matrix<T, Traits::N_states, Traits::N_states>;
    using ControlsVector = Eigen::Matrix<T, Traits::N_controls, 1>;
    using ControlsMatrix = Eigen::Matrix<T, Traits::N_states, Traits::N_controls>;
    using MeasurementVector = Eigen::Matrix<T, Traits::N_meas, 1>;
    using MeasurementMatrix = Eigen::Matrix<T, Traits::N_meas, Traits::N_meas>;
    using MeasurementStatesMatrix = Eigen::Matrix<T, Traits::N_meas, Traits::N_states>;
    using StatesMeasurementMatrix = Eigen::Matrix<T, Traits::N_states, Traits::N_meas>;

    KalmanFilter() = default;
    virtual ~KalmanFilter() = default;
    
    void update (double dt) {
      // Perform matrix updates based on dt.
      updateProcessMatrices(dt);

      // Calculate new estimate with the process model.
      estimate_ = (states_T_states__process_ * estimate_) + (states_T_controls__process_ * controls_);

      // Calculate the new covariance.
      estimate_covariance_ =
	(states_T_states__process_ * estimate_covariance_ * states_T_states__process_.transpose()) +
	process_cov_;
    }

    void add_measurement (MeasurementVector meas, MeasurementMatrix meas_cov, MeasurementStatesMatrix meas_T_states) {
      // Calculate the Kalman gain, based on the current covariance.
      auto states_T_meas = meas_T_states.transpose();
      auto gain = (estimate_covariance_ * states_T_meas) /
	((meas_T_states * estimate_covariance_ * states_T_meas) + meas_cov);

      // Calculate the measurement in terms of the states
      auto meas_estimate = states_T_meas * meas + meas_processing_noise__states_;

      // Update the estimate of the system based on the measurement
      estimate_ = estimate_ + (gain * (meas_estimate - meas_T_states * estimate_));

      // Update covariance of estimate
      estimate_covariance_ = (ProcessMatrix::Identity() - gain * meas_T_states) * estimate_covariance_;
    }
    
    void initialize (StatesVector initial_estimate, ControlsVector control_params) {
      estimate_ = std::move(initial_estimate);
      controls_ = std::move(control_params);
    }

    const StatesVector estimate () const {
      return estimate_;
    }

    const ProcessMatrix covariance() const {
      return estimate_covariance_;
    }

  protected:
    // Update the internal process matrices with the
    // provided delta-time value
    virtual void updateProcessMatrices(double dt) = 0;
    
    // The current estimate of the states in the system.
    // Initialization of this class will establish this
    // value during construction.
    StatesVector estimate_ = StatesVector::Zero();

    // The current error level of the estimate.
    ProcessMatrix estimate_covariance_ = ProcessMatrix::Identity();
    
    // The control factors for the model.
    // Additional factors that are constant throughout
    // the process and vary with time.
    ControlsVector controls_ = ControlsVector::Zero();

    // The process matrix. Determines how the dt update is
    // applied to an open-loop process.
    ProcessMatrix states_T_states__process_ = ProcessMatrix::Identity();

    // The controls matrix. Determines how the dt update
    // is applied to the process with the control values.
    ControlsMatrix states_T_controls__process_ = ControlsMatrix::Zero();

    // The process covariance matrix. Determines a flat amount
    // of noise floor added to the covariance to prevent
    // the kalman gain from becoming latched at 0 and
    // refusing to contribute measurement information.
    ProcessMatrix process_cov_ = ProcessMatrix::Zero();

    // The measurement processing error (of the device,
    // e.g. internal electrical noise in analog device)
    StatesVector meas_processing_noise__states_ = StatesVector::Zero();

  };
  
}
