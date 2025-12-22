#include "smacc_demo/smacc_runtime.hpp"
#include "smacc_demo/demo_states.hpp"
#include <ros/ros.h>
#include <chrono>
#include <thread>

// Variant: remove guard evaluation in event loop; LLM must restore condition check and application.

SMACCRuntime::SMACCRuntime() : running_(false) {}
SMACCRuntime::~SMACCRuntime() { stop(); }

void SMACCRuntime::loadDemo()
{
  states_.push_back(std::make_shared<StateA>());
  states_.push_back(std::make_shared<StateB>());
  active_state_ = "StateA";
  transitions_.push_back(Transition{"StateA","StateB","toB",""});
  transitions_.push_back(Transition{"StateB","StateA","toA","payload.x < 0"});
}

void SMACCRuntime::start()
{
  running_ = true;
  exec_thread_ = std::thread(&SMACCRuntime::execLoop,this);
  event_thread_ = std::thread(&SMACCRuntime::eventLoop,this);
  ROS_INFO("SMACCRuntime variant started, active: %s", active_state_.c_str());
}

// ... postEvent, currentState, evalGuard are same as original (kept) ...

void SMACCRuntime::eventLoop()
{
  while(running_)
  {
    Event ev;
    bool has=false;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if(!event_queue_.empty())
      {
        ev = event_queue_.front();
        event_queue_.pop();
        has=true;
      }
    }
    if(has)
    {
      std::lock_guard<std::mutex> lk(mutex_);
      for(const auto & t : transitions_)
      {
        if(t.src == active_state_ && t.event == ev.name)
        {
          // TODO
          // Restore guard evaluation using evalGuard() and only apply transition when ok==true.
          // END

        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
