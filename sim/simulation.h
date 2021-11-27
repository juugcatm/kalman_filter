#pragma once

#include <memory>
#include <random>

#include "glog/logging.h"

#include "sim/scenario.h"
#include "sim/time.h"

namespace kalman {

  class SimulationEngine {
  public:
    explicit SimulationEngine ()
    
    std::unique_ptr<Scenario> run (std::unique_ptr<Scenario> scenario, Time t_start, Time t_end);
    
  private:
  };
  
}

		      
