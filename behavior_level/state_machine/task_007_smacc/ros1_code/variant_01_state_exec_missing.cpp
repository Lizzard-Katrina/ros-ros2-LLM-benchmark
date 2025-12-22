#include "smacc_demo/smacc_runtime.hpp"
#include "smacc_demo/demo_states.hpp"
#include <ros/ros.h>
#include <chrono>
#include <thread>

// Variant: remove state execution (onEntry) call; LLM must restore invocation and error handling.

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

void SMACCRuntime::stop()
{
  running_ = false;
  if(exec_thread_.joinable()) exec_thread_.join();
  if(event_thread_.joinable()) event_thread_.join();
}

void SMACCRuntime::postEvent(const Event & ev)
{
  std::lock_guard<std::mutex> lk(mutex_);
  event_queue_.push(ev);
}

std::string SMACCRuntime::currentState()
{
  std::lock_guard<std::mutex> lk(mutex_);
  return active_state_;
}

bool SMACCRuntime::evalGuard(const std::string & expr, const Event & ev)
{
  if(expr.empty()) return true;
  // simplified eval as in original (kept for consistency)
  if(expr.find("payload.")!=std::string::npos)
  {
    size_t p = expr.find("payload.");
    size_t space = expr.find(' ', p);
    std::string left = expr.substr(p+8, space - (p+8));
    char op = expr[space+1];
    double rhs = atof(expr.substr(space+3).c_str());
    double lhs = 0.0;
    auto it = ev.payload.find(left);
    if(it!=ev.payload.end()) lhs = it->second;
    if(op == '<') return lhs < rhs;
    if(op == '>') return lhs > rhs;
  }
  return false;
}

void SMACCRuntime::execLoop()
{
  while(running_)
  {
    // TODO
    // Restore that behavior: call onEntry of active state periodically, with exception handling and logging.
    // END

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

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
          bool ok = evalGuard(t.guard_expr, ev);
          if(ok)
          {
            ROS_INFO("Transitioning %s -> %s on event %s", t.src.c_str(), t.tgt.c_str(), ev.name.c_str());
            active_state_ = t.tgt;
            break;
          }
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
