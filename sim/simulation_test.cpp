#include "sim/simulation.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

#include "sim/event.h"
#include "sim/time.h"

namespace kalman {

  class TickEvent : public Event {
  public:
    std::optional<EventInvocationResult>
    invoke (Scenario& scenario, Time t_current, std::default_random_engine& rng) const {
      times_called_++;
      return EventInvocationResult(periodic_);
    }

  private:
    std::shared_ptr<Periodic> periodic_;
    int times_called_;
  };
  
  TEST(Simulation, works) {
    constexpr Time kStart              (0);
    constexpr Time kEnd    (1'000'000'000);
    cosntexpr Time kPeriod   (100'000'000);
    // Generate a simulation engine with a deterministic
    // seed value for reliable test results.
    auto engine = [](){
      constexpr int kSeed = 42;
      std::default_random_engine random_engine(kSeed);
      return SimulationEngine(std::move(random_engine));
    }();

    // Define a scenario for the test.
    Scenario test_scenario;

    // Add events to the scenario.
    test_scenario.events().push(std::make_unique<TickEvent>(kPeriod), kStart);
    
    // Run the engine for the duration of the
    // scenario definition.
    engine.run(test_scenario, kStart, kEnd);

    // Make assertions of the scenario state
    // after the engine has run for the duration.
    
  }
  
}
