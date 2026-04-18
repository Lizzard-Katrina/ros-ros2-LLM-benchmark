#include <smacc/smacc_asynchronous_client_behavior.h>
#include <chrono>

namespace smacc
{
void SmaccAsyncClientBehavior::executeOnEntry()
{
  RCLCPP_INFO_STREAM(this->getLogger(), "[" << getName() << "] Creating asynchronous onEntry thread");
  this->onEntryThread_ = std::async(std::launch::async, [=] {
    this->onEntry();
    this->postFinishEventFn_();
    return 0;
  });
}

void SmaccAsyncClientBehavior::executeOnExit()
{
  RCLCPP_DEBUG_STREAM(
    this->getLogger(),
    "[" << getName() << "] Waiting for asynchronous onEntry thread to finish before onExit");

  rclcpp::Rate rate(100);
  auto status = this->onEntryThread_.wait_for(std::chrono::milliseconds(0));

  while (rclcpp::ok() && status != std::future_status::ready)
  {
    rate.sleep();
    status = this->onEntryThread_.wait_for(std::chrono::milliseconds(0));
  }

  try
  {
    this->onEntryThread_.get();
    RCLCPP_DEBUG_STREAM(
      this->getLogger(), "[" << getName() << "] Asynchronous onEntry thread joined successfully");
  }
  catch (...)
  {
    RCLCPP_DEBUG_STREAM(
      this->getLogger(),
      "[" << getName() << "] onEntry thread join skipped/already finished or raised during join");
  }
}

void SmaccAsyncClientBehavior::dispose()
{
  RCLCPP_DEBUG_STREAM(
    this->getLogger(),
    "[" << getName()
         << "] Destroying client behavior- Waiting finishing of asynchronous onExit thread");
  try
  {
    this->onExitThread_.get();
  }
  catch (...)
  {
    RCLCPP_DEBUG(
      this->getLogger(),
      "[SmaccAsyncClientBehavior] trying to Join onExit function, but it was already finished.");
  }

  RCLCPP_DEBUG_STREAM(
    this->getLogger(),
    "[" << getName()
         << "] Destroying client behavior-  onExit thread finished. Proccedding destruction.");
}

SmaccAsyncClientBehavior::~SmaccAsyncClientBehavior()
{
}

void SmaccAsyncClientBehavior::postSuccessEvent()
{
  postSuccessEventFn_();
}

void SmaccAsyncClientBehavior::postFailureEvent()
{
  postFailureEventFn_();
}

}  // namespace smacc