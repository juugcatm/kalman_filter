#pragma once

#include <utility>

#include <Eigen/Dense>
#include "glog/logging.h"

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
      LOG(INFO) << "Updated estimate: " << estimate_;
      
      // Calculate the new covariance.
      LOG(INFO) << "Stored cov: " << estimate_covariance_;
      ProcessMatrix est (states_T_states__process_ * estimate_covariance_.asDiagonal() * states_T_states__process_.transpose());
      LOG(INFO) << "Estimated cov: " << est;
      ProcessMatrix cov (process_cov_.asDiagonal());
      LOG(INFO) << "Process err: " << cov;
      estimate_covariance_ = (est + cov).diagonal();
      LOG(INFO) << "Updated cov: " << estimate_covariance_;
    }

    void add_measurement (MeasurementVector meas, MeasurementVector meas_cov, MeasurementStatesMatrix meas_T_states) {
      // Calculate the Kalman gain, based on the current covariance.
      auto states_T_meas = meas_T_states.transpose();
      MeasurementMatrix meas_cov_diag = meas_cov.asDiagonal();
      ProcessMatrix gain = (estimate_covariance_.asDiagonal() * states_T_meas);
      ProcessMatrix gain_denominator = ((meas_T_states * estimate_covariance_.asDiagonal() * states_T_meas) + meas_cov_diag);
      LOG(INFO) << "Gain numerator: " << gain;
      LOG(INFO) << "Gain denominator: " << gain_denominator;

      for (int r = 0; r < gain_denominator.rows(); r++) {
	for (int c = 0; c < gain_denominator.cols(); c++) {
	  gain(r, c) = (gain(r, c) == 0 ? 0 : gain(r, c) / gain_denominator(r, c));
	}
      }
      LOG(INFO) << "Gain mtx: " << gain;
      
      // Calculate the measurement in terms of the states
      auto meas_estimate = states_T_meas * meas + meas_processing_noise__states_;

      // Update the estimate of the system based on the measurement
      estimate_ = estimate_ + (gain * (meas_estimate - meas_T_states * estimate_));

      // Update covariance of estimate
      estimate_covariance_ = (ProcessMatrix::Identity() - gain * meas_T_states) * estimate_covariance_;
    }
    
    void initialize (StatesVector initial_estimate, ControlsVector control_params, StatesVector process_cov) {
      estimate_ = std::move(initial_estimate);
      controls_ = std::move(control_params);
      estimate_covariance_ = std::move(process_cov);
    }

    const StatesVector estimate () const {
      return estimate_;
    }

    const StatesVector covariance() const {
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
    StatesVector estimate_covariance_ = StatesVector::Identity();
    
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
    StatesVector process_cov_ = StatesVector::Zero();

    // The measurement processing error (of the device,
    // e.g. internal electrical noise in analog device)
    StatesVector meas_processing_noise__states_ = StatesVector::Zero();

  };
  
}
