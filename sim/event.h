#pragma once

#include <memory>

#include "sim/scenario.h"
#incldue "sim/schedule.h"
#include "sim/time.h"

namespace kalman {

  class Periodic {
  public:
    explicit Periodic (Duration period, std::unique_ptr<Generator<Duration>> jitter):
      period_(std::move(period)), jitter_(std::move(jitter)) {}
    
    Time next (Time t_current, std::default_random_engine& rng) const {
      return t_current + period_ + jitter_->next(rng);
    }
    
  private:
    Duration period_;
    std::unique_ptr<Generator<Duration>> jitter_;
  };
  
  class Event {
  public:
    virtual ~Event() = default;
    virtual std::optional<EventInvocationResult>
    invoke (Scenario& scenario, Time t_current, std::default_random_engine& rng) const = 0;
  };
   
  class ScheduledEvent {
  public:
    ScheduledEvent (std::unique_ptr<Event> event,
		    std::unique_ptr<Schedule> schedule):
      event_(std::move(event)),
      schedule_(std::move(schedule)) {}

    
    
    std::unique_ptr<const Event>& action() {
      return action_;
    }

    const std::unique_ptr<const Event>& action() const {
      return action_;
    }
		    
    const Time& deadline() const {
      return deadline_;
    }

    bool operator< (const ScheduledEvent& other) const {
      return this->deadline_ < other.deadline_;
    }
    
  private:
    Time deadline_;
    std::unique_ptr<const Event> action_;	    
  };
  
}

		     
