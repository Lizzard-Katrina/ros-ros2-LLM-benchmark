#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Messages for getting model and link poses
geometry_msgs::msg::Pose ball_model_pose, ball_link_pose;

// Service client for setting model poses
rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_model_state_client;
rclcpp::Node::SharedPtr node;

int getIndex(std::vector<std::string> v, std::string value)
{
    for(size_t i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr model_states)
{
    int ball_model_index = getIndex(model_states->name, "ball");
    if (ball_model_index != -1) {
        ball_model_pose = model_states->pose[ball_model_index];
    }
}

void link_states_callback(const gazebo_msgs::msg::LinkStates::SharedPtr link_states)
{
    int ball_link_index = getIndex(link_states->name, "ball::body");
    if (ball_link_index != -1) {
        ball_link_pose = link_states->pose[ball_link_index];
    }
}

void set_model_state(std::string model_name, 
                    std::string reference_frame, 
                    geometry_msgs::msg::Pose pose, 
                    geometry_msgs::msg::Twist model_twist)
{
    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = model_name;
    request->state.reference_frame = reference_frame;
    request->state.pose = pose;
    request->state.twist = model_twist;

    while (!set_model_state_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
    }

    auto result = set_model_state_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Successfully set model state.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service set_entity_state.");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("gazebo_model_states_node");

    auto model_sub = node->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states", 10, model_states_callback);
    auto link_sub = node->create_subscription<gazebo_msgs::msg::LinkStates>(
        "/gazebo/link_states", 10, link_states_callback);

    set_model_state_client = node->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
  
    geometry_msgs::msg::Pose model_pose;
    model_pose.position.x = 1.0;
    model_pose.position.y = 0.0;
    model_pose.position.z = 0.5;
    model_pose.orientation.w = 1.0;

    geometry_msgs::msg::Twist model_twist;
    model_twist.linear.x = 0.0;
    model_twist.angular.z = 0.0;

    // Set model state in Gazebo
    set_model_state("ball", "world", model_pose, model_twist);
  
    rclcpp::shutdown();
    return 0;
}