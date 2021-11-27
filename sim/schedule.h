#pragma once

namespace kalman {

  class Schedule {
  public:
    // Creates a single execution schedule which only suggests
    // execution once at the deadline time.
    static std::unique_ptr<Schedule> makeOneShot  (Time deadline);

    // Creates a periodic execution schedule which suggests execution
    // at the deadline, and every interval of period afterward.
    static std::unique_ptr<Schedule> makePeriodic (Time deadline, Duration period);

    // Updates the state of this schedule
    // Returns whether the schedule's state was updated.
    // A false return means no action need be taken
    // with this object going forward.
    bool advance();
    
  private:
    // Private construction. Use the helper functions instead
    // to create the appropriate tasks.
    Schedule (Time deadline, std::unique_ptr<const Priodic> periodic);

    // The time at which the execution should occur.
    Time deadline_;

    // The periodicity of the schedule. If not periodic,
    // this can be left as nullptr.
    std::unique_ptr<const Periodic> periodic_;
  };

}
