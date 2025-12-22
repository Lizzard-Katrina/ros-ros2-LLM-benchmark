#pragma once
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "tm_driver/tm_driver.h"
#include "tm_msgs/SendScript.h"
#include "tm_msgs/SetPositions.h"
#include "tm_msgs/AskSta.h"

class TmRosNode {
public:
    TmRosNode(const std::string &host);
    ~TmRosNode();
    void halt();

private:
    ////////////////////////////////
    // Action
    ////////////////////////////////
    void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);
    void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);

    ////////////////////////////////
    // Service
    ////////////////////////////////
    //TODO: write 3 bool that: send script to robot and set response
    //send joint positions to robot and set response
    // request status from robot and set response
    // END
};
