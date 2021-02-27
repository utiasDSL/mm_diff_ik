#pragma once

#include <ros/ros.h>

namespace mm {

// Abstract base class for trajectories.
class Trajectory {
 public:
  Trajectory()
      : index(0), is_finite(true), is_initialized(false), is_started(false) {}
  ~Trajectory() {}

  virtual bool start(const ros::Time& time) {
    if (!initialized()) {
      return false;
    }
    start_time = time.toSec();
    is_started = true;
    return true;
  }

  virtual bool finished(const ros::Time& time) {
    // Infinite trajectories don't end
    if (!is_finite) {
      return false;
    }
    // Other trajectories end once their duration is expired
    return time.toSec() > start_time + duration;
  }

  bool initialized() { return is_initialized; }
  bool started() { return is_started; }

 protected:
  double start_time;
  double duration;

  size_t index;
  bool is_finite;
  bool is_initialized;
  bool is_started;
};  // class Trajectory

}  // namespace mm
