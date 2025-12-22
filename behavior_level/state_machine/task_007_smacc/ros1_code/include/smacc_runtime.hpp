#ifndef SMACC_DEMO_SMACC_RUNTIME_HPP
#define SMACC_DEMO_SMACC_RUNTIME_HPP

#include <ros/ros.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include <map>
#include <functional>

struct Event
{
  std::string name;
  std::map<std::string, double> payload; // simple payload for demonstration
};

class IState
{
public:
  virtual ~IState() {}
  virtual void onEntry() = 0;
  virtual void onExit() = 0;
  virtual std::string name() const = 0;
};

using StatePtr = std::shared_ptr<IState>;

struct Transition
{
  std::string src;
  std::string tgt;
  std::string event;
  std::string guard_expr; // simplified guard (C++ expr in string) for demo
};

class SMACCRuntime
{
public:
  SMACCRuntime();
  ~SMACCRuntime();

  void loadDemo(); // load demo states and transitions
  void start();
  void stop();

  void postEvent(const Event & ev);

  // public for testing
  std::string currentState();
  std::vector<StatePtr> states_;
  std::vector<Transition> transitions_;

private:
  void execLoop();
  void eventLoop();
  std::mutex mutex_;
  std::thread exec_thread_;
  std::thread event_thread_;
  bool running_;
  std::queue<Event> event_queue_;
  std::string active_state_;
  ros::NodeHandle nh_;

  // helper for guard eval
  bool evalGuard(const std::string & expr, const Event & ev);
};

#endif // SMACC_DEMO_SMACC_RUNTIME_HPP
