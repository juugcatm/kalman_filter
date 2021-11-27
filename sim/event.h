#pragma once

#include <memory>
#include <optional>

#include "sim/generator.h"
#include "sim/scenario.h"
#include "sim/time.h"

namespace kalman {

  class Scenario;
  
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
  
  class EventInvocationResult {
  public:
    EventInvocationResult (std::shared_ptr<const Periodic> periodic):
      periodic_(std::move(periodic)) {}
    
    std::shared_ptr<const Periodic> periodic() const {
      return periodic_;
    }
    
  private:
    std::shared_ptr<const Periodic> periodic_;
  };
  
  class Event {
  public:
    virtual ~Event() = default;
    virtual std::optional<EventInvocationResult>
    invoke (Scenario& scenario, Time t_current, std::default_random_engine& rng) const = 0;
  };
   
  class ScheduledEvent {
  public:
    ScheduledEvent (std::unique_ptr<const Event> action, Time deadline):
      deadline_(deadline),
      action_(std::move(action)) {}

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

		     
