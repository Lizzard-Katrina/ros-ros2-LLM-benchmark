#include <rclcpp/rclcpp.hpp>
#include <rsm_core/RobotControlMux.h>
#include <chrono>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rsm::RobotControlMux>();

    double update_frequency = 20.0;
    node->declare_parameter<double>("update_frequency", 20.0);
    node->get_parameter("update_frequency", update_frequency);

    auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_frequency));

    auto timer = node->create_wall_timer(
        timer_period,
        [node]() { node->publishTopics(); });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}