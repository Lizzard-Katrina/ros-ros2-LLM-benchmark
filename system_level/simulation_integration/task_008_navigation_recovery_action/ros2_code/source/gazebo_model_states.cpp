#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/srv/set_model_state.hpp>
#include <gazebo_msgs/msg/model_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Messages for getting model and link poses
geometry_msgs::msg::Pose ball_model_pose, ball_link_pose;

// Service client for setting model poses
rclcpp::Client<gazebo_msgs::srv::SetModelState>::SharedPtr set_model_state_client;

// Global node pointer so set_model_state can spin
rclcpp::Node::SharedPtr g_node;

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < (int)v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void model_states_callback(gazebo_msgs::msg::ModelStates::SharedPtr model_states)
{
    int ball_model_index = getIndex(model_states->name, "ball");
    if (ball_model_index >= 0)
        ball_model_pose = model_states->pose[ball_model_index];
}

void link_states_callback(gazebo_msgs::msg::LinkStates::SharedPtr link_states)
{
    int ball_link_index = getIndex(link_states->name, "ball::body");
    if (ball_link_index >= 0)
        ball_link_pose = link_states->pose[ball_link_index];
}

void set_model_state(std::string model_name,
                    std::string reference_frame,
                    geometry_msgs::msg::Pose pose,
                    geometry_msgs::msg::Twist model_twist)
{
    // Set model service request
    auto request = std::make_shared<gazebo_msgs::srv::SetModelState::Request>();

    // Model state msg
    gazebo_msgs::msg::ModelState modelstate;
    modelstate.model_name = model_name;
    modelstate.reference_frame = reference_frame;
    modelstate.pose = pose;
    modelstate.twist = model_twist;

    request->model_state = modelstate;

    // Wait for the service to become available
    if (!set_model_state_client->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(rclcpp::get_logger("model_states_handler"),
                     "Service /gazebo/set_model_state not available after waiting.");
        return;
    }

    // Call the service asynchronously
    auto result_future = set_model_state_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(g_node, result_future, std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("model_states_handler"), "Setting position of " << model_name << " model was successful.");
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("model_states_handler"), "Setting position of " << model_name << " model was failed.");
    }
}

int main(int argc, char **argv)
{
    // Create ROS2 node
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("model_states_handler");

    // Create subscribers for Gazebo model and link states
    auto model_states_subscriber = g_node->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states", 100, model_states_callback);
    auto link_states_subscriber = g_node->create_subscription<gazebo_msgs::msg::LinkStates>(
        "/gazebo/link_states", 100, link_states_callback);

    // Create service client for setting Gazebo model state
    set_model_state_client = g_node->create_client<gazebo_msgs::srv::SetModelState>("/gazebo/set_model_state");

    // Pose and velocity for model state setting
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

    // Set model state in Gazebo
    set_model_state("ball", "world", model_pose, model_twist);

    return 0;
}