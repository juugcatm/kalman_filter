#pragma once

#include <utility>

namespace kalman {

  template <typename T>
  class KalmanFilter {
  public:
    KalmanFilter(const T& meas, const T& meas_error)
      : estimate_(std::move(meas)), error_(std::move(meas_error)), gain_(1.0) {}
    
    T update (const T& meas, const T& meas_error) {
      gain_ = error_ / (error_ + meas_error);
      estimate_ = estimate_ + (gain_ * (meas - estimate_));
      error_ = (1 - gain_) * error_;
      return estimate_;
    }
    
    T estimate() const {
      return estimate_;
    }

    T error() const {
      return error_;
    }
    
    double gain() const {
      return gain_;
    }
    
  private:    
    T estimate_;
    T error_;
    double gain_;
    bool initialized_;
  };
  
}
