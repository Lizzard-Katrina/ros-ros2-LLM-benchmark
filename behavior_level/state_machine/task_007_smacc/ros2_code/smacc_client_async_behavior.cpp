#include <smacc/smacc_asynchronous_client_behavior.h>
#include <rclcpp/rclcpp.hpp>

namespace smacc
{
    void SmaccAsyncClientBehavior::executeOnEntry()
    {
        RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Creating asynchronous onEntry thread");
        this->onEntryThread_ = std::async(std::launch::async,
                                          [=] {
                                              this->onEntry();
                                              this->postFinishEventFn_();
                                              return 0;
                                          });
    }

    void SmaccAsyncClientBehavior::executeOnExit()
    {
        RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Joining asynchronous onEntry thread");
        rclcpp::Rate rate(10);
        while (rclcpp::ok() && this->onEntryThread_.valid() && this->onEntryThread_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        {
            rate.sleep();
        }
    }

    void SmaccAsyncClientBehavior::dispose()
    {
        RCLCPP_DEBUG_STREAM(getLogger(), "[" << getName() << "] Destroying client behavior- Waiting finishing of asynchronous onExit thread");
        try
        {
            this->onExitThread_.get();
        }
        catch (...)
        {
            RCLCPP_DEBUG(getLogger(), "[SmaccAsyncClientBehavior] trying to Join onExit function, but it was already finished.");
        }

        RCLCPP_DEBUG_STREAM(getLogger(), "[" << getName() << "] Destroying client behavior-  onExit thread finished. Proccedding destruction.");
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

} // namespace smacc