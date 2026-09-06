/*
 * A minimal test node that uses the standalone UKF implementation
 * (not depending on robot_localization library) to test projectSigmaPoint.
 */
#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "standalone_ukf.hpp"

using namespace robot_localization;

class UkfTestNode : public rclcpp::Node
{
public:
  UkfTestNode()
  : Node("ukf_test_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "ukf_test_output", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&UkfTestNode::run_test, this));
  }

private:
  void run_test()
  {
    // Only run once
    timer_->cancel();

    StandaloneUkf ukf;
    ukf.setConstants(0.001, 0.0, 2.0);

    // Access the filter state and set initial values
    Eigen::VectorXd state(STATE_SIZE);
    state.setZero();

    // Set some initial velocities in body frame
    state(StateMemberVx) = 1.0;   // 1 m/s forward
    state(StateMemberVy) = 0.0;
    state(StateMemberVz) = 0.0;

    // Set yaw to pi/4 (45 degrees)
    state(StateMemberYaw) = M_PI / 4.0;
    // Set pitch to pi/6 (30 degrees)
    state(StateMemberPitch) = M_PI / 6.0;
    // Set roll to 0
    state(StateMemberRoll) = 0.0;

    // Set some acceleration
    state(StateMemberAx) = 0.5;

    // Set angular velocity
    state(StateMemberVyaw) = 0.1;
    state(StateMemberVpitch) = 0.05;
    state(StateMemberVroll) = 0.02;

    // Set the filter state
    ukf.setState(state);

    // Set a reasonable covariance
    Eigen::MatrixXd cov(STATE_SIZE, STATE_SIZE);
    cov.setIdentity();
    cov *= 0.01;
    ukf.setEstimateErrorCovariance(cov);

    // Predict with dt = 0.1s
    rclcpp::Time ref_time(0, 0, RCL_ROS_TIME);
    rclcpp::Duration delta = rclcpp::Duration::from_seconds(0.1);

    ukf.predict(ref_time, delta);

    // Get the predicted state
    const Eigen::VectorXd & predicted = ukf.getState();

    // Publish the result
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.resize(STATE_SIZE);
    for (int i = 0; i < STATE_SIZE; ++i) {
      msg.data[i] = predicted(i);
    }
    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "UKF test completed. Published predicted state.");
    RCLCPP_INFO(this->get_logger(), "X=%.6f Y=%.6f Z=%.6f",
      predicted(StateMemberX), predicted(StateMemberY), predicted(StateMemberZ));
    RCLCPP_INFO(this->get_logger(), "Roll=%.6f Pitch=%.6f Yaw=%.6f",
      predicted(StateMemberRoll), predicted(StateMemberPitch), predicted(StateMemberYaw));
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UkfTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}