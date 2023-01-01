#ifndef RT_SCHEDULER_H
#define RT_SCHEDULER_H

// #include <time.h>

// C++
#include <thread>

namespace realtime_utils
{

typedef __int64_t timevar;

class RTClock
{
  public:
  bool *kill_this_node_;

  RTClock() {}
  RTClock(int cyclic_time_usec) { cyclic_time_usec_ = cyclic_time_usec; }

  void Init()
  {
    cycle_time_ = GetTimeUsec();
    now_time_ = cycle_time_;
    last_time_ = cycle_time_;
  }

  void Reset()
  {
    cycle_time_ = GetTimeUsec();
    now_time_ = cycle_time_;
    last_time_ = cycle_time_;
  }

  void SleepToCompleteCycle()
  {
    cycle_time_ += cyclic_time_usec_;
    now_time_ = GetTimeUsec();
    timevar time_to_sleep_us = cycle_time_ - now_time_;
    if (time_to_sleep_us < 0)
      Reset();
    else
      std::this_thread::sleep_for(std::chrono::microseconds(time_to_sleep_us));
  }

  timevar GetTimeUsec()
  {
    struct timespec tick;
    clock_gettime(CLOCK_MONOTONIC, &tick);
    return (tick.tv_sec * 1000000LLU) + (tick.tv_nsec / 1000);
  }

  private:
  int cyclic_time_usec_;

  timevar now_time_;
  timevar last_time_;
  timevar cycle_time_;
};
} // namespace realtime_utils

#endif
