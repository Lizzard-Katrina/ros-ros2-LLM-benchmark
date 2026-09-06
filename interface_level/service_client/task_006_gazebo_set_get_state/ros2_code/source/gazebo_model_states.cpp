#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/srv/set_model_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Messages for getting model and link poses
geometry_msgs::msg::Pose ball_model_pose, ball_link_pose;

int getIndex(std::vector<std::string> v, std::string value)
{
    for (int i = 0; i < (int)v.size(); i++)
    {
        if (v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr model_states)
{
    int ball_model_index = getIndex(model_states->name, "ball");
    if (ball_model_index >= 0) {
        ball_model_pose = model_states->pose[ball_model_index];
    }
}

void link_states_callback(const gazebo_msgs::msg::LinkStates::SharedPtr link_states)
{
    int ball_link_index = getIndex(link_states->name, "ball::body");
    if (ball_link_index >= 0) {
        ball_link_pose = link_states->pose[ball_link_index];
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS2 and create a node.
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("model_states_handler");

    // Create subscriptions and register callbacks.
    auto model_states_subscriber = node->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states", 100, model_states_callback);
    auto link_states_subscriber = node->create_subscription<gazebo_msgs::msg::LinkStates>(
        "/gazebo/link_states", 100, link_states_callback);

    // Create a service client
    auto set_model_state_client = node->create_client<gazebo_msgs::srv::SetModelState>(
        "/gazebo/set_model_state");

    // Populate pose and twist values for the model state request.
    geometry_msgs::msg::Pose model_pose;
    model_pose.position.x = 0;
    model_pose.position.y = 0;
    model_pose.position.z = 1;
    model_pose.orientation.x = 0.0;
    model_pose.orientation.y = 0.0;
    model_pose.orientation.z = 0.0;
    model_pose.orientation.w = 0.0;

    geometry_msgs::msg::Twist model_twist;
    model_twist.linear.x = 0.0;
    model_twist.linear.y = 0.0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;

    // Wait for service availability
    while (!set_model_state_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service. Exiting.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for service /gazebo/set_model_state to become available...");
    }

    // Build a service request
    auto request = std::make_shared<gazebo_msgs::srv::SetModelState::Request>();
    request->model_state.model_name = "ball";
    request->model_state.reference_frame = "world";
    request->model_state.pose = model_pose;
    request->model_state.twist = model_twist;

    // Send the request asynchronously
    auto future = set_model_state_client->async_send_request(request);

    // Use a dedicated executor to spin until the future completes
    auto result_code = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10));

    if (result_code == rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(node->get_logger(), "Setting position of ball model was successful.");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Setting position of ball model was failed.");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service /gazebo/set_model_state");
    }

    // Clean up subscriptions before shutdown to avoid RMW errors
    model_states_subscriber.reset();
    link_states_subscriber.reset();
    set_model_state_client.reset();
    node.reset();

    rclcpp::shutdown();
    return 0;
}