#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <future>
#include <chrono>
#include <string>
#include <functional>

// We re-declare the class here to avoid header dependency issues.
// In a real project this would come from the smacc2 header.
namespace smacc
{

class SmaccAsyncClientBehavior
{
public:
    virtual ~SmaccAsyncClientBehavior();
    virtual std::string getName() const { return "SmaccAsyncClientBehavior"; }
    virtual rclcpp::Logger getLogger() const { return rclcpp::get_logger(getName()); }
    virtual void onEntry() {}
    virtual void onExit() {}
    void executeOnEntry();
    void executeOnExit();
    void dispose();
    void postSuccessEvent();
    void postFailureEvent();

    std::function<void()> postFinishEventFn_;
    std::function<void()> postSuccessEventFn_;
    std::function<void()> postFailureEventFn_;
    std::future<int> onEntryThread_;
    std::future<int> onExitThread_;
};

} // namespace smacc

// A concrete test behavior
class TestBehavior : public smacc::SmaccAsyncClientBehavior
{
public:
    std::string entry_result;
    std::string exit_result;

    void onEntry() override
    {
        entry_result = "entry_done";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void onExit() override
    {
        exit_result = "exit_done";
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_async_node");

    auto pub = node->create_publisher<std_msgs::msg::String>("smacc_test_result", 10);

    TestBehavior behavior;
    bool finish_event_called = false;

    behavior.postFinishEventFn_ = [&finish_event_called]() {
        finish_event_called = true;
    };
    behavior.postSuccessEventFn_ = []() {};
    behavior.postFailureEventFn_ = []() {};

    // Execute the lifecycle
    behavior.executeOnEntry();
    behavior.executeOnExit();
    behavior.dispose();

    // Build result string
    std_msgs::msg::String msg;
    msg.data = "entry=" + behavior.entry_result +
               ";exit=" + behavior.exit_result +
               ";finish_event=" + (finish_event_called ? "true" : "false");

    // Publish result a few times to ensure subscriber gets it
    rclcpp::Rate rate(10);
    auto start = node->now();
    while (rclcpp::ok() && (node->now() - start).seconds() < 3.0)
    {
        pub->publish(msg);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}