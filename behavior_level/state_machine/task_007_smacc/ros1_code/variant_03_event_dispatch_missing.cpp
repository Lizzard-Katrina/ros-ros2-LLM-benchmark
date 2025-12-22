#include "smacc_demo/smacc_runtime.hpp"
#include "smacc_demo/demo_states.hpp"
#include <ros/ros.h>
#include <chrono>
#include <thread>

// Variant: remove event queue consumption and dispatch loop.
// LLM must restore thread-safe queue pop, matching transitions and applying guard checks.

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

void SMACCRuntime::postEvent(const Event & ev)
{
  std::lock_guard<std::mutex> lk(mutex_);
  event_queue_.push(ev);
}

void SMACCRuntime::eventLoop()
{
  // TODO
  // The original eventLoop repeatedly:
  //  - try to pop an event from event_queue_ under mutex
  //  - if got event, iterate transitions for active_state_ and evaluate guard via evalGuard()
  //  - if matched and ok, set active_state_ = t.tgt and log the transition
  //  - sleep a short duration to avoid busy waiting
  // END

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}
