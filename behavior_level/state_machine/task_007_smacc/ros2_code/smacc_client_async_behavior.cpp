#include <smacc/smacc_asynchronous_client_behavior.h>
#include <rclcpp/rclcpp.hpp>

namespace smacc
{
    void SmaccAsyncClientBehavior::executeOnEntry()
    {
        RCLCPP_INFO(this->getLogger(), "[%s] Creating asynchronous onEntry thread", getName().c_str());
        this->onEntryThread_ = std::async(std::launch::async,
                                          [=] {
                                              this->onEntry();
                                              this->postFinishEventFn_();
                                              return 0;
                                          });
    }

    void SmaccAsyncClientBehavior::executeOnExit()
    {
        RCLCPP_INFO(this->getLogger(), "[%s] Joining asynchronous onEntry thread", getName().c_str());
        rclcpp::Rate rate(10);
        while (this->onEntryThread_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
        {
            rate.sleep();
            if (!rclcpp::ok(this->getNode()->get_node_base_interface()))
            {
                RCLCPP_WARN(this->getLogger(), "[%s] ROS2 is shutting down, stopping wait for onEntryThread_", getName().c_str());
                break;
            }
        }
        try
        {
            this->onEntryThread_.get();
        }
        catch (...)
        {
            RCLCPP_WARN(this->getLogger(), "[%s] Exception caught while joining onEntryThread_", getName().c_str());
        }
    }

    void SmaccAsyncClientBehavior::dispose()
    {
        RCLCPP_DEBUG(this->getLogger(), "[%s] Destroying client behavior- Waiting finishing of asynchronous onExit thread", getName().c_str());
        try
        {
            this->onExitThread_.get();
        }
        catch (...)
        {
            RCLCPP_DEBUG(this->getLogger(), "[SmaccAsyncClientBehavior] trying to Join onExit function, but it was already finished.");
        }

        RCLCPP_DEBUG(this->getLogger(), "[%s] Destroying client behavior-  onExit thread finished. Proccedding destruction.", getName().c_str());
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