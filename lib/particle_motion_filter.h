#pragma once

#include <Eigen/Dense>

#include "lib/kalman_filter.h"
#include "lib/types.h"

namespace kalman {

  /* A specialization of the KalmanFilter which tracks
   * a particle (orientation-less) with position, velocity
   * and acceleration.
   */
  template <typename T>
  class ParticleMotionFilter {
  public:
    ParticleMotionFilter () {
      filter_.initialize({0, 0, 0}, {0, 0, 0});
    }

    /* Advance the particle motion process with the provided
     * delta_t value.
     */
    void update (double dt, Eigen::Matrix<T, 3, 1> Q = Eigen::Matrix<T, 3, 1>::Zero()) {
      // Standard 2nd-degree polynomial motion model
      Eigen::Matrix<T, 3, 3> A = Eigen::Matrix<T, 3, 3>::Identity();
      A(0, 1) = dt;
      A(0, 2) = 0.5 * pow(dt, 2);
      A(1, 2) = dt;

      // Update the base class
      filter_.update(A, Q);
    }
    
    /* Provide updated measurements to the filter */
    void add_measurements (const std::optional<MeasurementWithVariance<T>>& position,
			   const std::optional<MeasurementWithVariance<T>>& velocity,
			   const std::optional<MeasurementWithVariance<T>>& acceleration) {
      
      Eigen::Matrix<T, 3, 3> measurements_T_states = Eigen::Matrix<T, 3, 3>::Zero();
      Eigen::Matrix<T, 3, 1> measurements = Eigen::Matrix<T, 3, 1>::Zero();
      Eigen::Matrix<T, 3, 1> measurements_variance = Eigen::Matrix<T, 3, 1>::Zero();

      if (position) {
	measurements_T_states(0, 0) = 1;
	measurements[0] = position->value();
	measurements_variance[0] = position->variance();
      }

      if (velocity) {
	measurements_T_states(1, 1) = 1;
	measurements[1] = velocity->value();
	measurements_variance[1] = velocity->variance();	
      }

      if (acceleration) {
	measurements_T_states(2, 2) = 1;
	measurements[2] = acceleration->value();
	measurements_variance[2] = acceleration->variance();
      }
      
      filter_.template add_measurement<3>(std::move(measurements_T_states),
					  std::move(measurements),
					  std::move(measurements_variance));
    }

    const T& position() const {
      return filter_.estimate()[0];
    }

    const T& velocity() const {
      return filter_.estimate()[1];
    }

    const T& acceleration() const {
      return filter_.estimate()[2];
    }
    
  private:
    KalmanFilter<T, 3> filter_;
  };

  
}
