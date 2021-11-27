#pragma once

#include <cstdint>
#include <ctime>

namespace kalman {

  class Duration {
  public:
    explicit Duration (uint64_t duration_ns):
      duration_ns_(duration_ns) {}
    
    operator uint64_t() const {
      return duration_ns_;
    }
    
  private:
    uint64_t duration_ns_;
  };

  class Time {
  public:
    static constexpr uint64_t kNSecPerSec = 1'000'000'000;

    constexpr explicit Time (int time_ns):
      time_ns_(time_ns) {}
    
    constexpr explicit Time (uint64_t time_ns):
      time_ns_(time_ns) {}

    constexpr explicit Time (time_t time):
      time_ns_(uint64_t(time) * kNSecPerSec) {}

    Duration operator- (const Time& other) {
      return Duration(static_cast<uint64_t>(*this) - static_cast<uint64_t>(other));
    }

    Time operator+ (const Duration& other) {
      return Time(static_cast<uint64_t>(*this) + static_cast<uint64_t>(other));
    }
    
    operator uint64_t () const {
      return time_ns_;
    }
    
  private:
    uint64_t time_ns_;
  };
  
}
