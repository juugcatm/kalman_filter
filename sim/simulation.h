#pragma once

#include <random>

#include "glog/logging.h"

#include "sim/scenario.h"
#include "sim/time.h"

namespace kalman {

  class SimulationEngine {
  public:
    explicit SimulationEngine (std::default_random_engine random_engine)
      : random_engine_(std::move(random_engine)) {}
    
    Scenario run (Scenario scenario, Time t_start, Time t_end) {
      // Borrow the queue of events from the scenario
      // and begin executing them.
      auto& scheduled_events = scenario.events();

      // Initialize the scenario at the start time and begin
      // executing.
      Time t_current = t_start;
      while (t_current < t_end && scheduled_events.size()) {
	// Fetch the upcoming event record,
	// removing it from the priority queue.
	auto scheduled_event = [&scheduled_events](){
	  // Safely implement a move from the top element in the queue by using a const cast
	  // and poping the element at the same time.
	  auto e = std::move(const_cast<ScheduledEvent&>(scheduled_events.top()));
	  scheduled_events.pop();
	  return e;
	}();

	// Advance the continuous properties of
	// the scenario from the previous time to the
	// current time.
	scenario.advance(Duration(scheduled_event.deadline() - t_current));

	// Invoke the next event in the scenario
	// which will potentially mutate the scenario
	// state, returning an EventInvocationResult.
	auto result = scheduled_event.action()->invoke(scenario, t_current, random_engine_);
	if (!result) {
	  LOG(ERROR) << "Failed invocation of event!";
	  // TODO: LOG(ERROR) << result;
	  return scenario;
	}

	// If the result is periodic, reschedule for the
	// future deadline.
	if (result->periodic()) {
	  scheduled_events.emplace(std::move(scheduled_event.action()), result->periodic()->next(t_current, random_engine_));
	}

	// Update the simulation time
	t_current = scheduled_event.deadline();
      }	

      // Advance the remaining scenario state
      if (t_current < t_end) {
	scenario.advance(Duration(t_end - t_current));
      }
      
      return scenario;
    };
    
  private:

    // random engine
    std::default_random_engine random_engine_;
    
  };
  
}

		      
