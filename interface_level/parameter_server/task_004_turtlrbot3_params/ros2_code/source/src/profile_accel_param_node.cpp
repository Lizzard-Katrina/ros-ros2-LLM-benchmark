// Executable reference for the parameter-event migration exercised by
// task_004_turtlrbot3_params.
//
// The verbatim turtlebot3.cpp cannot be compiled or run here: it pulls in the
// whole turtlebot3_node package, DynamixelSDK and OpenCR hardware. This node
// carries the *exact* migrated logic from
//   TurtleBot3::init_dynamixel_sdk_wrapper() / TurtleBot3::parameter_event_callback()
// so that the ROS1->ROS2 parameter-event conversion pattern is genuinely built
// and executed by tests/test_runtime_ros2.py:
//
//   * observe parameter changes with rclcpp::AsyncParametersClient
//   * wait_for_service(1s) before wiring the subscription
//   * on_parameter_event() callback (NOT add_on_set_parameters_callback)
//   * iterate event->changed_parameters
//   * on "motors.profile_acceleration": new value is DIVIDED by
//     motors_.profile_acceleration_constant for the rev/min2 unit conversion
//
// The converted value is republished on ~/profile_acceleration_converted so a
// test can observe the result of the callback.

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

class ProfileAccelParamNode : public rclcpp::Node
{
public:
  ProfileAccelParamNode()
  : rclcpp::Node("profile_accel_param_node")
  {
    // Mirrors TurtleBot3::add_motors()
    this->declare_parameter<double>("motors.profile_acceleration_constant", 214.577);
    this->declare_parameter<double>("motors.profile_acceleration", 0.0);

    profile_acceleration_constant_ =
      this->get_parameter("motors.profile_acceleration_constant").as_double();
    profile_acceleration_ =
      this->get_parameter("motors.profile_acceleration").as_double();

    converted_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "~/profile_acceleration_converted", 10);

    // Mirrors the migrated body of init_dynamixel_sdk_wrapper() /
    // parameter_event_callback().
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);

    if (!parameters_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Parameter service not available");
      if (!rclcpp::ok()) {
        return;
      }
    }

    param_event_sub_ = parameters_client_->on_parameter_event(
      [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
      {
        for (const auto & changed_parameter : event->changed_parameters) {
          if (changed_parameter.name == "motors.profile_acceleration") {
            auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter);
            profile_acceleration_ =
              value.as_double() / profile_acceleration_constant_;
            RCLCPP_INFO(
              this->get_logger(),
              "motors.profile_acceleration is changed : %f rev/min2",
              profile_acceleration_);

            std_msgs::msg::Float64 out;
            out.data = profile_acceleration_;
            converted_pub_->publish(out);
          }
        }
      }
    );

    RCLCPP_INFO(this->get_logger(), "profile_accel_param_node ready");
  }

private:
  double profile_acceleration_constant_ {214.577};
  double profile_acceleration_ {0.0};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr converted_pub_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProfileAccelParamNode>());
  rclcpp::shutdown();
  return 0;
}
