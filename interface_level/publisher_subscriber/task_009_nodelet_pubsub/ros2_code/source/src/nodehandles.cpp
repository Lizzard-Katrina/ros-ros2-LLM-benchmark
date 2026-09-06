#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <string>

namespace test_nodelet
{

class NodehandleTest : public rclcpp::Node
{
public:
  explicit NodehandleTest(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("nodehandle_test", options)
  {
    global_pub_ = this->create_publisher<builtin_interfaces::msg::Time>("/global", 1000);
    namespaced_pub_ = this->create_publisher<std_msgs::msg::Byte>("namespaced", 1000);
    private_pub_ = this->create_publisher<std_msgs::msg::Bool>("private", 1000);
  }

private:
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr global_pub_;
  rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr namespaced_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr private_pub_;
};

}  // namespace test_nodelet

RCLCPP_COMPONENTS_REGISTER_NODE(test_nodelet::NodehandleTest)