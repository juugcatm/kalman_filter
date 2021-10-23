#pragma once

#include <utility>

#include <Eigen/Dense>

namespace kalman {

  template <typename T, typename Traits>
  class KalmanFilter {
  public:
    using StatesVector = Eigen::Matrix<T, Traits::N_states, 1>;
    using ProcessMatrix = Eigen::Matrix<T, Traits::N_states, Traits::N_states>;
    
    KalmanFilter() {}
    
    void update (double dt) {
      // Perform matrix updates based on dt.
      update_A(dt);

      // Calculate new estimate with the process model.
      estimate_ = A_ * estimate_;
    }

    void initialize (StatesVector initial_estimate) {
      estimate_ = std::move(initial_estimate);
    }

    const StatesVector estimate () const {
      return estimate_;
    }

  private:
    void update_A (double dt) {
      A_(0, 1) = dt;
    }

    // The current estimate of the states in the system.
    // Initialization of this class will establish this
    // value during construction.
    StatesVector estimate_ = StatesVector::Zero();

    // The process matrix. Determines how the dt update is
    // applied to an open-loop process.
    ProcessMatrix A_ = ProcessMatrix::Identity();
    
  };
  
}
