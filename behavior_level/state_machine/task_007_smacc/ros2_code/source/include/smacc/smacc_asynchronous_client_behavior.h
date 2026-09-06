#ifndef SMACC_ASYNCHRONOUS_CLIENT_BEHAVIOR_H
#define SMACC_ASYNCHRONOUS_CLIENT_BEHAVIOR_H

#include <rclcpp/rclcpp.hpp>
#include <future>
#include <functional>
#include <string>
#include <cxxabi.h>

namespace smacc
{

// Minimal stub so the .cpp compiles in isolation.
inline std::string demangleSymbol(const char* name)
{
    int status = 0;
    char* demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status);
    std::string result = (status == 0 && demangled) ? demangled : name;
    free(demangled);
    return result;
}

class SmaccAsyncClientBehavior
{
public:
    virtual ~SmaccAsyncClientBehavior();

    virtual void executeOnEntry();
    virtual void executeOnExit();
    virtual void dispose();

    void postSuccessEvent();
    void postFailureEvent();

    virtual std::string getName() const
    {
        return demangleSymbol(typeid(*this).name());
    }

    virtual rclcpp::Logger getLogger() const
    {
        return rclcpp::get_logger(getName());
    }

    // User-overridable hooks
    virtual void onEntry() {}
    virtual void onExit() {}

    // Callbacks set by the framework
    std::function<void()> postFinishEventFn_ = [](){};
    std::function<void()> postSuccessEventFn_ = [](){};
    std::function<void()> postFailureEventFn_ = [](){};

protected:
    std::future<int> onEntryThread_;
    std::future<int> onExitThread_;
};

} // namespace smacc

#endif // SMACC_ASYNCHRONOUS_CLIENT_BEHAVIOR_H