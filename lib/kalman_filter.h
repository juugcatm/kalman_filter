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
    
    KalmanFilter() {}
    
    void update (double dt) {
      // Perform matrix updates based on dt.
      update_A(dt);
      update_B(dt);
      
      // Calculate new estimate with the process model.
      estimate_ = (A_ * estimate_) + (B_ * controls_);
    }

    void initialize (StatesVector initial_estimate, ControlsVector control_params) {
      estimate_ = std::move(initial_estimate);
      controls_ = std::move(control_params);
    }

    const StatesVector estimate () const {
      return estimate_;
    }

  private:
    void update_A (double dt) {
      A_(0, 1) = dt;
    }

    void update_B (double dt) {
      B_(0, 0) = 0.5 * pow(dt, 2);
      B_(1, 0) = dt;
    }
    
    // The current estimate of the states in the system.
    // Initialization of this class will establish this
    // value during construction.
    StatesVector estimate_ = StatesVector::Zero();

    // The control factors for the model.
    // Additional factors that are constant throughout
    // the process and vary with time.
    ControlsVector controls_ = ControlsVector::Zero();
    
    // The process matrix. Determines how the dt update is
    // applied to an open-loop process.
    ProcessMatrix A_ = ProcessMatrix::Identity();

    // The controls matrix. Determines how the dt update
    // is applied to the process with the control values.
    ControlsMatrix B_ = ControlsMatrix::Zero();
    
  };
  
}
