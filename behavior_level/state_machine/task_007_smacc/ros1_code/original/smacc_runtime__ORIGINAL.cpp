// ORIGINAL runtime (simplified, runnable demo)
#include "smacc_demo/smacc_runtime.hpp"
#include "smacc_demo/demo_states.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>

// Simple guard evaluator: supports expressions like "payload.x > 0"
static double getPayloadValue(const Event& ev, const std::string & key)
{
  auto it = ev.payload.find(key);
  if(it != ev.payload.end()) return it->second;
  return 0.0;
}

SMACCRuntime::SMACCRuntime() : running_(false)
{
  // nothing
}

SMACCRuntime::~SMACCRuntime()
{
  stop();
}

void SMACCRuntime::loadDemo()
{
  // create two demo states
  states_.push_back(std::make_shared<StateA>());
  states_.push_back(std::make_shared<StateB>());
  active_state_ = "StateA";

  // transitions: StateA --(toB)--> StateB ; StateB --(toA if payload.x<0)--> StateA
  transitions_.push_back(Transition{"StateA","StateB","toB",""});
  transitions_.push_back(Transition{"StateB","StateA","toA","payload.x < 0"});
}

void SMACCRuntime::start()
{
  running_ = true;
  exec_thread_ = std::thread(&SMACCRuntime::execLoop,this);
  event_thread_ = std::thread(&SMACCRuntime::eventLoop,this);
  ROS_INFO("SMACC runtime started, active state: %s", active_state_.c_str());
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

// evaluate guard expression (VERY simplified for demo)
bool SMACCRuntime::evalGuard(const std::string & expr, const Event & ev)
{
  if(expr.empty()) return true;
  // very naive parsing for expressions like "payload.x < 0" or "payload.x > 1.5"
  if(expr.find("payload.")!=std::string::npos)
  {
    size_t p = expr.find("payload.");
    size_t space = expr.find(' ', p);
    std::string left = expr.substr(p+8, space - (p+8)); // 'x'
    char op = expr[space+1];
    double rhs = atof(expr.substr(space+3).c_str());
    double lhs = getPayloadValue(ev,left);
    if(op == '<') return lhs < rhs;
    if(op == '>') return lhs > rhs;
    return false;
  }
  return false;
}

void SMACCRuntime::execLoop()
{
  while(running_)
  {
    // call onEntry of active_state periodically (simple demo)
    {
      std::lock_guard<std::mutex> lk(mutex_);
      for(auto & s : states_)
      {
        if(s->name() == active_state_)
        {
          try
          {
            ROS_INFO("Calling onEntry of %s", s->name().c_str());
            s->onEntry();
          }
          catch(const std::exception & e)
          {
            ROS_ERROR("Exception in state onEntry: %s", e.what());
          }
          break;
        }
      }
    }
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
      // match transitions from active_state
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
