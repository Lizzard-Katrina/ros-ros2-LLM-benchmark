# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

// Messages for getting model and link poses
geometry_msgs::Pose ball_model_pose, ball_link_pose;

// Service client for setting model poses
ros::ServiceClient set_model_state_client;

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void model_states_callback(gazebo_msgs::ModelStates model_states)
{
    int ball_model_index = getIndex(model_states.name, "ball");
    ball_model_pose = model_states.pose[ball_model_index];
}

void link_states_callback(gazebo_msgs::LinkStates link_states)
{
    int ball_link_index = getIndex(link_states.name, "ball::body");
    ball_link_pose = link_states.pose[ball_link_index];
}

void set_model_state(std::string model_name, 
                    std::string reference_frame, 
                    geometry_msgs::Pose pose, 
                    geometry_msgs::Twist model_twist)
{
    // TODO: Build a service request to set the model state using the given inputs.

    // handle shutdown cases.

    // Send the request asynchronously and wait for completion.

    // Log success/failure.
    // END of TODO
}

}

int main() {
    // TODO:
    // fill this main function so that it
    //Initialize ROS2 and create a node.
    // Create subscriptions and register callbacks.
    // Create a service client
    // Populate pose and twist values for the model state request.
    // Invoke the model state update routine
  
    geometry_msgs::Pose model_pose;

    geometry_msgs::Twist model_twist;
    //END OF TODO

    // Set model state in Gazebo
    set_model_state("ball", "world", model_pose, model_twist);
  
    return 0;
}

----------------------------


# LLM Response

 ```cpp
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/srv/set_model_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Messages for getting model and link poses
geometry_msgs::msg::Pose ball_model_pose, ball_link_pose;

// Service client for setting model poses
rclcpp::Client<gazebo_msgs::srv::SetModelState>::SharedPtr set_model_state_client;
rclcpp::Node::SharedPtr g_node;

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
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

void set_model_state(std::string model_name, 
                    std::string reference_frame, 
                    geometry_msgs::msg::Pose pose, 
                    geometry_msgs::msg::Twist model_twist)
{
    // TODO: Build a service request to set the model state using the given inputs.
    auto request = std::make_shared<gazebo_msgs::srv::SetModelState::Request>();
    request->model_state.model_name = model_name;
    request->model_state.reference_frame = reference_frame;
    request->model_state.pose = pose;
    request->model_state.twist = model_twist;

    // handle shutdown cases.
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(g_node->get_logger(), "ROS2 is not running");
        return;
    }

    // Send the request asynchronously and wait for completion.
    auto future = set_model_state_client->async_send_request(request);
    
    // Wait for the result
    auto return_code = rclcpp::spin_until_future_complete(g_node, future, std::chrono::seconds(5));
    
    // Log success/failure.
    if (return_code == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(g_node->get_logger(), "Set model state succeeded: %s", response->status_message.c_str());
        } else {
            RCLCPP_ERROR(g_node->get_logger(), "Set model state failed: %s", response->status_message.c_str());
        }
    } else {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to call service set_model_state");
    }
    // END of TODO
}

int main(int argc, char **argv) {
    // TODO:
    // fill this main function so that it
    //Initialize ROS2 and create a node.
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("set_model_state_node");
    
    // Create subscriptions and register callbacks.
    auto model_states_sub = g_node->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states", 10, model_states_callback);
    auto link_states_sub = g_node->create_subscription<gazebo_msgs::msg::LinkStates>(
        "/gazebo/link_states", 10, link_states_callback);
    
    // Create a service client
    set_model_state_client = g_node->create_client<gazebo_msgs::srv::SetModelState>("/gazebo/set_model_state");
    
    // Wait for service to be available
    while (!set_model_state_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(g_node->get_logger(), "Interrupted while waiting for service");
            return 1;
        }
        RCLCPP_INFO(g_node->get_logger(), "Waiting for service...");
    }

    // Populate pose and twist values for the model state request.
    geometry_msgs::msg::Pose model_pose;
    model_pose.position.x = 0.0;
    model_pose.position.y = 0.0;
    model_pose.position.z = 0.5;
    model_pose.orientation.x = 0.0;
    model_pose.orientation.y = 0.0;
    model_pose.orientation.z = 0.0;
    model_pose.orientation.w = 1.0;

    geometry_msgs::msg::Twist model_twist;
    model_twist.linear.x = 0.0;
    model_twist.linear.y = 0.0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;
    //END OF TODO

    // Set model state in Gazebo
    set_model_state("ball", "world", model_pose, model_twist);
    
    rclcpp::shutdown();
    return 0;
}
```