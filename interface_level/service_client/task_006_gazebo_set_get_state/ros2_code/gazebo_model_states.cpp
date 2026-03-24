Here is the converted ROS2 code:

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
    ball_model_pose = model_states->pose[ball_model_index];
}

void link_states_callback(const gazebo_msgs::msg::LinkStates::SharedPtr link_states)
{
    int ball_link_index = getIndex(link_states->name, "ball::body");
    ball_link_pose = link_states->pose[ball_link_index];
}

void set_model_state(std::string model_name, 
                    std::string reference_frame, 
                    geometry_msgs::msg::Pose pose, 
                    geometry_msgs::msg::Twist model_twist)
{
    auto request = std::make_shared<gazebo_msgs::srv::SetModelState_Request>();
    request->model_state.model_name = model_name;
    request->model_state.pose = pose;
    request->model_state.twist = model_twist;
    request->model_state.reference_frame = reference_frame;

    auto future = set_model_state_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(set_model_state_client->get_node()->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(set_model_state_client->get_node()->get_logger(), "Model state set successfully");
    }
    else
    {
        RCLCPP_ERROR(set_model_state_client->get_node()->get_logger(), "Failed to set model state");
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("set_model_state_node");

    // Create subscriptions and register callbacks
    auto model_states_sub = node->create_subscription<gazebo_msgs::msg::ModelStates>("gazebo/model_states", 10, model_states_callback);
    auto link_states_sub = node->create_subscription<gazebo_msgs::msg::LinkStates>("gazebo/link_states", 10, link_states_callback);

    // Create a service client
    set_model_state_client = node->create_client<gazebo_msgs::srv::SetModelState>("gazebo/set_model_state");

    // Populate pose and twist values for the model state request
    geometry_msgs::msg::Pose model_pose;
    model_pose.position.x = 1.0;
    model_pose.position.y = 2.0;
    model_pose.position.z = 3.0;

    geometry_msgs::msg::Twist model_twist;
    model_twist.linear.x = 1.0;
    model_twist.linear.y = 2.0;
    model_twist.linear.z = 3.0;

    // Invoke the model state update routine
    set_model_state("ball", "world", model_pose, model_twist);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

Note that I've used the `rclcpp` package to create a ROS2 node, and the `gazebo_msgs` package to interact with Gazebo. I've also used the `geometry_msgs` package to work with pose and twist messages.