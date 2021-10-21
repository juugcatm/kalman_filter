#include "kalman_filter.h"

#include <random>

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace kalman {

  TEST(KalmanFilter, works) {
    const double kSigma = 1.0;
    
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> rand (5.0, kSigma);

    std::unique_ptr<KalmanFilter<double>> filter;
    for (std::size_t i = 0; i < 50; i++) {
      double meas = rand(gen);
      if (!filter) {
	filter.reset(new KalmanFilter<double>(meas, kSigma));
      } else {
	filter->update(meas, kSigma);
      }
      LOG(INFO) << "Updated with meas=" << meas
		<< " now estimate=" << filter->estimate()
		<< " with error=" << filter->error()
		<< " and gain=" << filter->gain();
    }
  }
  
}
