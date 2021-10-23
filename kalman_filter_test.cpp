#include "kalman_filter.h"

#include <random>

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace kalman {

  /*
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
  */
  
  class RisingFluidTraits {
  public:
    static constexpr std::size_t N_states = 2;
    static constexpr std::size_t N_controls = 0;
    static constexpr std::size_t N_meas = 0;
  };
  
  class KalmanFilterRisingFluidFixture
    : public ::testing::Test {
  public:
    using FilterType = KalmanFilter<double, RisingFluidTraits>;
    using StatesVector = FilterType::StatesVector;
    
    FilterType filter_;
  };
  
  TEST_F(KalmanFilterRisingFluidFixture, works) {
    StatesVector initial_state;
    initial_state[0] = 0;    // 0 m of position
    initial_state[1] = 1.0;  // 1 m/s velocity

    filter_.initialize(initial_state);

    filter_.update(0.1);
    EXPECT_EQ(filter_.estimate()[0], 0.1);
    EXPECT_EQ(filter_.estimate()[1], 1.0);

    filter_.update(1.0);
    EXPECT_EQ(filter_.estimate()[0], 1.1);
    EXPECT_EQ(filter_.estimate()[1], 1.0);
  }
 
}
