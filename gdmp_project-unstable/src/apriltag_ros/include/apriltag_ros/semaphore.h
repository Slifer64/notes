#ifndef APRILTAG_ROS_SEMAPHORE_H
#define APRILTAG_ROS_SEMAPHORE_H

#include <mutex>
#include <condition_variable>

namespace apriltag_ros
{

class Semaphore
{
private:
  std::mutex mutex_;
  std::condition_variable condition_;
  // unsigned long count_ = 0; // Initialized as locked.
  bool count_ = false;  // Initialized as locked.

public:

  void notify()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    // ++count_;
    count_ = true;
    condition_.notify_one();
  }

  void wait()
  {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    while(!count_) // Handle spurious wake-ups.
      condition_.wait(lock);
    // --count_;
    count_ = false;
  }

  bool wait_for(double time_ms)
  {
    std::unique_lock<decltype(mutex_)> lock(mutex_);

    int wait_time = time_ms * 1e6;

    while(!count_) // Handle spurious wake-ups.
    {
      if (condition_.wait_for(lock,std::chrono::nanoseconds(wait_time))==std::cv_status::timeout)
        return false;
    }
    // --count_;
    count_ = false;

    return true;
  }

  bool try_wait()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    if(count_)
    {
      // --count_;
      count_ = false;
      return true;
    }
    return false;
  }
  
}; // class Semaphore

} // apriltag_ros

#endif // APRILTAG_ROS_SEMAPHORE_H
