#pragma once

#include <iostream>
#include <shared_mutex>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "task_011_amr_interop/msg/error.hpp"
#include "task_011_amr_interop/msg/info.hpp"
#include "task_011_amr_interop/msg/load.hpp"
#include "task_011_amr_interop/msg/order_state.hpp"

namespace adapter
{

class SafeState
{
public:
  using OrderState = task_011_amr_interop::msg::OrderState;

  OrderState get() const
  {
    std::shared_lock lock(mutex);
    return order_state_;
  }

  template <class T, class U>
  void set_parameter(T OrderState::*member, U && value)
  {
    try {
      std::unique_lock lock(mutex);
      order_state_.*member = std::forward<U>(value);
    } catch (const std::exception & e) {
      std::cout << "Wrong assignment of value on order_state member: " << e.what() << std::endl;
    }
  }

  void add_information(const task_011_amr_interop::msg::Info & info)
  {
    std::unique_lock lock(mutex);
    order_state_.informations.push_back(info);
  }

  void add_load(const task_011_amr_interop::msg::Load & load)
  {
    std::unique_lock lock(mutex);
    order_state_.loads.push_back(load);
  }

  void add_error(const task_011_amr_interop::msg::Error & error)
  {
    std::unique_lock lock(mutex);
    order_state_.errors.push_back(error);
  }

  void clear()
  {
    std::unique_lock lock(mutex);
    order_state_.loads.clear();
    order_state_.informations.clear();
    order_state_.errors.clear();
  }

  void reset()
  {
    std::unique_lock lock(mutex);
    order_state_ = OrderState();
  }

private:
  OrderState order_state_;
  mutable std::shared_mutex mutex;
};

class Handler
{
public:
  Handler() {}

  virtual ~Handler() {}

  virtual void compose(
    rclcpp::Node * node, SafeState * current_state, const std::string & robot_name)
  {
    if (node == nullptr || current_state == nullptr) {
      throw std::runtime_error(
        "Either node or current_state pass pointers are nullptr when composing handler.");
    }
    node_ = node;
    current_state_ = current_state;
    robot_name_ = robot_name;
  }

  virtual void configure() = 0;

  virtual void execute() = 0;

protected:
  rclcpp::Node * node_{nullptr};
  SafeState * current_state_{nullptr};
  std::string robot_name_;
};

}  // namespace adapter