#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/srv/set_model_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

geometry_msgs::msg::Pose ball_model_pose;
geometry_msgs::msg::Pose ball_link_pose;

rclcpp::Client<gazebo_msgs::srv::SetModelState>::SharedPtr set_model_state_client;

int getIndex(const std::vector<std::string> & v, const std::string & value)
{
    for(size_t i = 0; i < v.size(); i++)
    {
        if(v[i] == value)
            return static_cast<int>(i);
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

void set_model_state(rclcpp::Node::SharedPtr node,
                     const std::string & model_name, 
                     const std::string & reference_frame, 
                     const geometry_msgs::msg::Pose & pose, 
                     const geometry_msgs::msg::Twist & model_twist)
{
    auto request = std::make_shared<gazebo_msgs::srv::SetModelState::Request>();

    gazebo_msgs::msg::ModelState modelstate;
    modelstate.model_name = model_name;
    modelstate.reference_frame = reference_frame;
    modelstate.pose = pose;
    modelstate.twist = model_twist;

    request->model_state = modelstate;

    auto result_future = set_model_state_client->async_send_request(request);

    // Wait for the result (blocking)
    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Setting position of %s model was successful.", model_name.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Setting position of %s model failed.", model_name.c_str());
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gazebo_model_state_updater");

    auto model_states_sub = node->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states", 10, model_states_callback);

    auto link_states_sub = node->create_subscription<gazebo_msgs::msg::LinkStates>(
        "/gazebo/link_states", 10, link_states_callback);

    set_model_state_client = node->create_client<gazebo_msgs::srv::SetModelState>("/gazebo/set_model_state");

    // Wait for the service to be available
    if (!set_model_state_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Service /gazebo/set_model_state not available.");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        geometry_msgs::msg::Twist zero_twist;
        zero_twist.linear.x = 0.0;
        zero_twist.linear.y = 0.0;
        zero_twist.linear.z = 0.0;
        zero_twist.angular.x = 0.0;
        zero_twist.angular.y = 0.0;
        zero_twist.angular.z = 0.0;

        set_model_state(node, "ball", "world", ball_model_pose, zero_twist);

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}