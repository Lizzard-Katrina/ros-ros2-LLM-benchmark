# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: gazebo_model_states.cpp
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
    // Set model service struct
    gazebo_msgs::SetModelState setmodelstate;

    // Model state msg
    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = model_name;
    modelstate.reference_frame = reference_frame;
    modelstate.pose = pose;
    modelstate.twist = model_twist;

    setmodelstate.request.model_state = modelstate;

    // Call the service
    bool success = set_model_state_client.call(setmodelstate);
    if (success)
    {
        ROS_INFO_STREAM("Setting position of " << model_name << "model was successful.");
    }
    else
    {
        ROS_ERROR_STREAM("Setting position of " << model_name << "model was failed.");
    }
}

int main()
{
    // ===== TODO BLOCK START =====
    // Implement logic to update Gazebo model 'ball' pose via subscription callbacks and service client
    // ===== TODO BLOCK END =====
  
    return 0;
}

----------------------------


# LLM Response

[FILENAME: gazebo_model_states.cpp]
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/srv/set_model_state.hpp>
#include <gazebo_msgs/msg/model_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// Messages for getting model and link poses
geometry_msgs::msg::Pose ball_model_pose, ball_link_pose;

// Service client for setting model poses
rclcpp::Client<gazebo_msgs::srv::SetModelState>::SharedPtr set_model_state_client;
rclcpp::Node::SharedPtr node;

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < static_cast<int>(v.size()); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void model_states_callback(gazebo_msgs::msg::ModelStates::SharedPtr model_states)
{
    int ball_model_index = getIndex(model_states->name, "ball");
    if (ball_model_index >= 0 && ball_model_index < static_cast<int>(model_states->pose.size()))
    {
        ball_model_pose = model_states->pose[ball_model_index];
    }
}

void link_states_callback(gazebo_msgs::msg::LinkStates::SharedPtr link_states)
{
    int ball_link_index = getIndex(link_states->name, "ball::body");
    if (ball_link_index >= 0 && ball_link_index < static_cast<int>(link_states->pose.size()))
    {
        ball_link_pose = link_states->pose[ball_link_index];
    }
}

void set_model_state(std::string model_name, 
                    std::string reference_frame, 
                    geometry_msgs::msg::Pose pose, 
                    geometry_msgs::msg::Twist model_twist)
{
    // Set model service struct
    auto request = std::make_shared<gazebo_msgs::srv::SetModelState::Request>();

    // Model state msg
    gazebo_msgs::msg::ModelState modelstate;
    modelstate.model_name = model_name;
    modelstate.reference_frame = reference_frame;
    modelstate.pose = pose;
    modelstate.twist = model_twist;

    request->model_state = modelstate;

    // Call the service
    if (!set_model_state_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Setting position of " << model_name << "model was failed.");
        return;
    }

    auto future = set_model_state_client->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5));

    if (ret == rclcpp::FutureReturnCode::SUCCESS && future.get()->success)
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "Setting position of " << model_name << "model was successful.");
    }
    else
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Setting position of " << model_name << "model was failed.");
    }
}

int main(int argc, char ** argv)
{
    // ===== TODO BLOCK START =====
    // Implement logic to update Gazebo model 'ball' pose via subscription callbacks and service client
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("gazebo_model_states");

    auto model_states_subscriber = node->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states",
        rclcpp::QoS(10),
        model_states_callback);

    auto link_states_subscriber = node->create_subscription<gazebo_msgs::msg::LinkStates>(
        "/gazebo/link_states",
        rclcpp::QoS(10),
        link_states_callback);

    set_model_state_client =
        node->create_client<gazebo_msgs::srv::SetModelState>("/gazebo/set_model_state");

    while (rclcpp::ok() && !set_model_state_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(node->get_logger(), "Waiting for /gazebo/set_model_state service...");
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::Rate rate(10.0);
    while (rclcpp::ok())
    {
        executor.spin_some();

        geometry_msgs::msg::Pose pose = ball_model_pose;
        geometry_msgs::msg::Twist twist;

        pose.position.x = ball_link_pose.position.x;
        pose.position.y = ball_link_pose.position.y;
        pose.position.z = ball_link_pose.position.z;
        pose.orientation = ball_link_pose.orientation;

        set_model_state("ball", "world", pose, twist);

        rate.sleep();
    }

    rclcpp::shutdown();
    // ===== TODO BLOCK END =====
  
    return 0;
}