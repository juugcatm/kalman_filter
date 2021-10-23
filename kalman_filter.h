#pragma once

#include <utility>

#include <Eigen/Dense>

namespace kalman {

  template <typename T, typename Traits>
  class KalmanFilter {
  public:
    KalmanFilter() {}
    
    void update () {}

  private:    
    Eigen::Matrix<T, Traits::N_states, 1> estimate_ =
      Eigen::Matrix<T, Traits::N_states, 1>::Zero();

  };
  
}
