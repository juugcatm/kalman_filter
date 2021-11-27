#pragma once

#include <queue>

#include "sim/event.h"
#include "sim/time.h"

namespace kalman {

  template <typename Derived>
  class Scenario {
  public:
    virtual ~Scenario() = default;

    // Advance the internal state of the scenario, based
    // only on elapsed time. This function is used to model
    // any sort of background process.
    virtual void advance (Duration duration) {}

    // Schedule an event with the provided scheduling parameters.
    void scheduleEvent (std::unique_ptr<Event<Derived>> event,
			std::unique_ptr<Schedule> schedule);
    
    std::priority_queue<ScheduledEvent>& events() {
      return events_;
    }

    const std::priority_queue<ScheduledEvent>& events() const {
      return events_;
    }

  private:
    std::priority_queue<ScheduledEvent> events_;
  };
  

}
