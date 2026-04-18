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
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

using gazebo_msgs::msg::LinkStates;
using gazebo_msgs::msg::ModelStates;
using gazebo_msgs::srv::SetModelState;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;

class GazeboModelStatesNode : public rclcpp::Node
{
public:
    GazeboModelStatesNode() : Node("gazebo_model_states_node")
    {
        ball_model_pose_ = std::make_shared<Pose>();
        ball_link_pose_ = std::make_shared<Pose>();

        model_states_sub_ = this->create_subscription<ModelStates>(
            "/model_states", 10,
            std::bind(&GazeboModelStatesNode::model_states_callback, this, std::placeholders::_1));

        link_states_sub_ = this->create_subscription<LinkStates>(
            "/link_states", 10,
            std::bind(&GazeboModelStatesNode::link_states_callback, this, std::placeholders::_1));

        set_model_state_client_ = this->create_client<SetModelState>("/set_model_state");

        while (!set_model_state_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for /set_model_state service...");
        }

        update_ball_pose();
    }

private:
    rclcpp::Subscription<ModelStates>::SharedPtr model_states_sub_;
    rclcpp::Subscription<LinkStates>::SharedPtr link_states_sub_;
    rclcpp::Client<SetModelState>::SharedPtr set_model_state_client_;
    std::shared_ptr<Pose> ball_model_pose_;
    std::shared_ptr<Pose> ball_link_pose_;

    int getIndex(std::vector<std::string> v, std::string value)
    {
        for (size_t i = 0; i < v.size(); i++)
        {
            if (v[i].compare(value) == 0)
                return static_cast<int>(i);
        }
        return -1;
    }

    void model_states_callback(const ModelStates::SharedPtr model_states)
    {
        int ball_model_index = getIndex(model_states->name, "ball");
        if (ball_model_index >= 0 && ball_model_index < static_cast<int>(model_states->pose.size())) {
            *ball_model_pose_ = model_states->pose[ball_model_index];
        }
    }

    void link_states_callback(const LinkStates::SharedPtr link_states)
    {
        int ball_link_index = getIndex(link_states->name, "ball::body");
        if (ball_link_index >= 0 && ball_link_index < static_cast<int>(link_states->pose.size())) {
            *ball_link_pose_ = link_states->pose[ball_link_index];
        }
    }

    void set_model_state(std::string model_name,
                         std::string reference_frame,
                         Pose pose,
                         Twist model_twist)
    {
        auto request = std::make_shared<SetModelState::Request>();
        request->model_state.model_name = model_name;
        request->model_state.reference_frame = reference_frame;
        request->model_state.pose = pose;
        request->model_state.twist = model_twist;

        auto future = set_model_state_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Setting position of " << model_name << " model was successful.");
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Setting position of " << model_name << " model failed.");
        }
    }

    void update_ball_pose()
    {
        Pose new_pose;
        new_pose.position.x = 1.0;
        new_pose.position.y = 0.5;
        new_pose.position.z = 0.2;
        new_pose.orientation.w = 1.0;

        Twist zero_twist;

        set_model_state("ball", "world", new_pose, zero_twist);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboModelStatesNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}