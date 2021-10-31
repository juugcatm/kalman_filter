#pragma once

#include <utility>

#include <Eigen/Dense>
#include "glog/logging.h"

namespace kalman {

  template <typename T, std::size_t N_s, std::size_t N_c, std::size_t N_m>
  class KalmanFilter {
  public:
    KalmanFilter() = default;
    virtual ~KalmanFilter() = default;

    virtual Eigen::Matrix<T, N_s, N_s> A (double dt) = 0;
    virtual Eigen::Matrix<T, N_s, N_c> B (double dt) = 0;
    virtual Eigen::Matrix<T, N_m, N_s> H () = 0;

    /**
     * Perform a process update on the state of the Kalman Filter.
     * This is used when the filter is being advanced between measurements.
     * Common use cases, for example, are advancing time from t -> t' between
     * application of measurements at t and t'. 
     */
    void update (double dt, Eigen::Matrix<T, N_c, 1> controls, Eigen::Matrix<T, N_s, 1> process_variance) {
      Eigen::Matrix<T, N_s, N_s> A_ = A(dt);
      Eigen::Matrix<T, N_s, N_c> B_ = B(dt);
      estimate_ = A_ * estimate_ + B_ * controls;
      estimate_variance_ = (A_ * estimate_variance_.asDiagonal() * A_.transpose()).diagonal() + (process_variance * dt);
    }

    /**
     * Perform a measurement update on the state of the Kalman Filter.
     * This is used when a new measurement arrives. The process of the filter
     * should be advanced to match this provided measurement before calling
     * this function.
     */
    void add_measurement (double dt, Eigen::Matrix<T, N_m, 1> measurement, Eigen::Matrix<T, N_m, 1> measurement_variance) {
      Eigen::Matrix<T, N_m, N_s> H_ = H();
      Eigen::Matrix<T, N_s, N_m> K = (estimate_covariance_ * H_.transpose()) *
	(H_ * estimate_covariance_ * H_.transpose() + measurement_variance.asDiagonal()).inverse();
      estimate_ = estimate_ + K * (measurement - H * estimate_);
      estimate_covariance_ = estimate_covariance_ - (K * H * estimate_covariance_.asDiagonal()).diagonal();
    }
        
    void initialize (Eigen::Matrix<T, N_s, 1> initial_estimate, Eigen::Matrix<T, N_s, 1> initial_estimate_cov) {
      estimate_ = std::move(initial_estimate);
      estimate_covariance_ = std::move(initial_estiamte_cov);
    }

    const Eigen::Matrix<T, N_s, 1> estimate () const {
      return estimate_;
    }

    const Eigen::Matrix<T, N_s, 1> covariance() const {
      return estimate_covariance_;
    }

  protected:
    // The current estimate of the states in the system.
    // Initialization of this class will establish this
    // value during construction.
    Eigen::Matrix<T, N_s, 1> estimate_ = Eigen::Matrix<T, N_s, 1>::Zero();

    // The current error level of the estimate.
    Eigen::Matrix<T, N_s, 1> estimate_covariance_ = Eigen::Matrix<T, N_s, 1>::Zero();
  };
  
}
