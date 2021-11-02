#pragma once

#include <tuple>

namespace kalman {

  template <typename T>
  class MeasurementWithVariance {
  public:
    MeasurementWithVariance (T value, T variance)
      : value_(std::move(value)),
	variance_(std::move(variance)) {}

    const T& value() const {
      return value_;
    }

    const T& variance() const {
      return variance_;
    }
    
  private:
    T value_;
    T variance_;
  };
  
}
