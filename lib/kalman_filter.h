#pragma once

#include <utility>

#include <Eigen/Dense>
#include "glog/logging.h"

namespace kalman {

  template <typename T, std::size_t N_s>
  class KalmanFilter {
  public:
    KalmanFilter() = default;
    virtual ~KalmanFilter() = default;

    /**
     * Perform a process update on the state of the Kalman Filter.
     * The user must provide the process matrix to perform the update
     * and the appropriate process noise. The matrix and the noise are
     * typically built based on the duration of the process advancement.
     * All control variables must be included in the process matrix
     * in advance.
     */
    void update (const Eigen::Matrix<T, N_s, N_s>& process_matrix,
		 const Eigen::Matrix<T, N_s, 1>&  process_variance) {
      estimate_ = process_matrix * estimate_;
      estimate_variance_ = (process_matrix * estimate_variance_.asDiagonal() * process_matrix.transpose()).diagonal() +
	process_variance;
    }

    /**
     * Perform a measurement update on the state of the Kalman Filter.
     * The user must provide the measurement values and their variances,
     * and additionally the 'H' matrix (meas_T_states) which maps the
     * measurement values into the states.
     */
    template <std::size_t N_m>
    void add_measurement (const Eigen::Matrix<T, N_m, N_s>& H,
			  const Eigen::Matrix<T, N_m, 1>& measurement,
			  const Eigen::Matrix<T, N_m, 1>& measurement_variance) {

      Eigen::Matrix<T, N_s, N_m> K;
      Eigen::Matrix<T, N_m, N_m> R = measurement_variance.asDiagonal();

      // Condition the problem for measurements with no variance.
      // This essentially sets the state of the problem based on the
      // measurement without any regard to the current process variance.
      if (measurement_variance.squaredNorm() == 0 ||
	  estimate_variance_.squaredNorm() == 0) {
	K = H.transpose();
      } else {
	K = (estimate_variance_.asDiagonal() * H.transpose()) *
	  (H * estimate_variance_.asDiagonal() * H.transpose() + R).inverse();
      }

      LOG(INFO) << "Numerator: " << estimate_variance_.asDiagonal() * H.transpose();
      LOG(INFO) << "Gain: " << K;
      
      estimate_ = estimate_ + K * (measurement - H * estimate_);
      estimate_variance_ = estimate_variance_ - (K * H * estimate_variance_.asDiagonal()).diagonal();

      LOG(INFO) << "Estimate variance: " << estimate_variance_;
    }
        
    void initialize (const Eigen::Matrix<T, N_s, 1>& initial_estimate,
		     const Eigen::Matrix<T, N_s, 1>& initial_estimate_variance) {
      estimate_ = initial_estimate;
      estimate_variance_ = initial_estimate_variance;
    }

    const Eigen::Matrix<T, N_s, 1>& estimate () const {
      return estimate_;
    }

    const Eigen::Matrix<T, N_s, 1>& variance() const {
      return estimate_variance_;
    }

  protected:
    // The current estimate of the states in the system.
    // Initialization of this class will establish this
    // value during construction.
    Eigen::Matrix<T, N_s, 1> estimate_ = Eigen::Matrix<T, N_s, 1>::Zero();

    // The current error level of the estimate.
    Eigen::Matrix<T, N_s, 1> estimate_variance_ = Eigen::Matrix<T, N_s, 1>::Zero();
  };
  
}
