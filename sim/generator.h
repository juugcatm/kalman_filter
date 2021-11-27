#pragma once

namespace kalman {

  template <typename T>
  class Generator {

  public:
    virtual T next (std::default_random_engine& random_engine) = 0;
    
  };
  
}
