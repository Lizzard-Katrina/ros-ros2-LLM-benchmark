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
