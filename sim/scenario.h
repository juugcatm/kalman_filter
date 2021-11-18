#pragma once

#include "sim/event.h"
#include "sim/time.h"

namespace kalman {

  class Scenario {
  public:
    void advance (Duration duration) {
    }
     
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
