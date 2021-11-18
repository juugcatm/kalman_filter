#pragma once

namespace kalman {

  class Time {
  public:
    constexpr std::uint64_t kNSecPerSec = 1'000'000'000;
    
    explicit Time (std::uint64_t time_ns):
      time_ns_(time_ns) {}

    explicit Time (std::time_t time):
      time_ns_(std::uint64_t(time) * kNSecPerSec) {}

    Duration operator-(const Time& other) {
      return Duration(std::unit64_t(*this) - std::unit64_t(other));
    }
    
    std::uint64_t operator(std::unit64_t) () {
      return time_ns_;
    }
    
  private:
    std::uint64_t time_ns_;
  };

  
  class Duration {
    explicit Duration (std::uint64_t duration_ns):
      duration_ns_(duration_ns) {}

    std::uint64_t operator(std::unit64_t) () {
      return duration_ns_;
    }

  private:
    std::uint64_t duration_ns_;
  };
  
}
