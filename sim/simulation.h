#pragma once

#include <random>

#include "sim/scenario.h"
#include "sim/time.h"

namespace kalman {

  class SimulationEngine {
  public:
    explicit SimulationEngine (std::some_random_engine rng)
      : rng_(std::move(rng)) {}
    
    Scenario run (Scenario scenario, Time t_start, Time t_end) {
      // Borrow the queue of events from the scenario
      // and begin executing them.
      auto& scheduled_events = scenario.events();

      // Initialize the scenario at the start time and begin
      // executing.
      for (Time t_current = t_start; t_current < t_end && scenario.events.size(); ) {
	// Fetch the upcoming event record,
	// removing it from the priority queue.
	auto scheduled_event = scheduled_events.pop_back();

	// Advance the continuous properties of
	// the scenario from the previous time to the
	// current time.
	scenario.advance(scheduled_event.deadline() - t_current);

	// Invoke the next event in the scenario
	// which will potentially mutate the scenario
	// state, returning an EventInvocationResult.
	auto result = scheduled_event.action()->invoke(scenario, t_current);
	if (!result) {
	  LOG(ERROR) << "Failed invocation of event!";
	  LOG(ERROR) << result;
	  return scenario;
	}

	// If the result is periodic, reschedule for the
	// future deadline.
	if (result->periodic()) {
	  scheduled_events.push_back(std::move(scheduled_event.action()), result->periodic()->next(t_current, rng_));
	}

	// Update the simulation time
	t_current = scheduled_event.deadline();
      }	

      // Advance the remaining scenario state
      scenario.advance(t_end - t_current);
      
      return scenario;
    };
    
  private:

    // random engine
    std::some_random_engine rng_;
    
  };
  
}

		      
