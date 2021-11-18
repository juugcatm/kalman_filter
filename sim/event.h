#pragma once

#include <memory>

#include "sim/generator.h"
#include "sim/time.h"

namespace kalman {

  class Periodic {
  public:
    explicit Periodic (Duration period, std::unique_ptr<Generator<Duration>> jitter)
    
    Time next (Time t_current, std::some_random_engine& rng) const {
      return t_current + period_ + jitter_->next(rng);
    }
    
  private:
    Duration period_;
    std::unique_ptr<Generator<Duration>> jitter_;
  };
  
  class EventInvocationResult {
  public:
    EventInvocationResult (std::shared_ptr<const Periodic> periodic):
      periodic_(std::move(periodic));
    
    std::shared_ptr<const Periodic> periodic() const {
      return periodic_;
    }
    
  private:
    std::shared_ptr<const Periodic> periodic_;
  };
  
  class Event {
  public:
    virtual ~Event() = default;
    virtual EventInvocationResult invoke (Scenario& scenario, Time t_current, std::some_random_engine& rng) = 0;
  };
   
  class ScheduledEvent {
  public:
    ScheduledEvent (std::unique_ptr<const Event> action, Time deadline):
      action_(std::move(action)),
      deadline_(deadline) {}

    std::unique_ptr<const Event>& action() {
      return action_;
    }

    const std::unique_ptr<const Event>& action() const {
      return action_;
    }
		    
    const Time& deadline() const {
      return deadline_;
    }
    
  private:
    Time deadline_;
    std::unique_ptr<const Event> action_;	    
  };
  
}

		     
