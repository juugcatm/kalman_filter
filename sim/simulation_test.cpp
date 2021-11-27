#include "sim/simulation.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

#include "sim/event.h"
#include "sim/time.h"

namespace kalman {

  class TestScenario : public Scenario<TestScenario> {
  public:
    void increment() { counter_++; }

    const int& counter() const { return counter_ }
    
  private:
    int counter_ = 0;
  };
  
  TEST(Simulation, works) {
    constexpr Time kStart (0);
    constexpr Time kEnd (1'000'000'000);
    constexpr Duration kPeriod (100'000'000);

    // Generate a simulation engine with a deterministic
    // seed value for reliable test results.
    auto engine = [](){
      /*
      constexpr int kSeed = 42;
      std::default_random_engine random_engine(kSeed);
      return SimulationEngine(std::move(random_engine));
      */
      return SimulationEngine();
    }();

    // Define a scenario for the test.
    auto test_scenario = std::make_unique<TestScenario>();

    // Add events to the scenario.
    test_scenario->scheduleEvent([](TestScenario& scenario, Time t_current){
      scenario.increment();
    },
    std::make_unique<PeriodicSchedule>(kStart, kPeriod));
    
    // Run the engine for the duration of the
    // scenario definition. We temporarily give ownership
    // of the scenario to the engine, who returns it back
    // to us at the end of the execution.
    test_scenario = engine.run(std::move(test_scenario), kStart, kEnd);

    // Make assertions of the scenario state
    // after the engine has run for the duration.
    ASSERT(test_scenario->counter(), 10);
  }
  
}
