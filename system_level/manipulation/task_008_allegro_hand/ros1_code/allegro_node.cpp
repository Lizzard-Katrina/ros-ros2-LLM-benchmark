// Common allegro node code used by any node. Each node that implements an
// AllegroNode must define the computeDesiredTorque() method.
//
// Editor: Hibo (sh-yang @ wonik.com)

#include "allegro_node.h"

std::string jointNames[DOF_JOINTS] =
        {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
                "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",
                "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0",
        };

std::vector<std::string> frame_ids = {"link_3_0_tip", "link_7_0_tip", "link_11_0_tip", "link_15_0_tip"};
std::vector<std::string> namespaces = {"fingertip_[0]", "fingertip_[1]", "fingertip_[2]", "fingertip_[3]"};
std::vector<visualization_msgs::Marker> markers(4);

AllegroNode::AllegroNode(bool sim /* = false */) {
  mutex = new boost::mutex();
  
  // Create arrays 16 long for each of the four joint state components
  current_joint_state.position.resize(DOF_JOINTS);
  current_joint_state.velocity.resize(DOF_JOINTS);
  current_joint_state.effort.resize(DOF_JOINTS);
  current_joint_state.name.resize(DOF_JOINTS);

  // Initialize values: joint names should match URDF, desired torque and
  // velocity are both zero.
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.name[i] = jointNames[i];
    desired_torque[i] = 0.0;
    current_velocity[i] = 0.0;
    current_position_filtered[i] = 0.0;
    current_velocity_filtered[i] = 0.0;
  }

  
  // Get Allegro Hand information from parameter server
  // This information is found in the Hand-specific "zero.yaml" file from the allegro_hand_description package
  std::string robot_name, manufacturer, origin, serial;
  double version;
  ros::param::get("~hand_info/robot_name", robot_name);
  ros::param::get("~hand_info/which_hand", whichHand);
  ros::param::get("~hand_info/which_type", whichType);
  ros::param::get("~hand_info/manufacturer", manufacturer);
  ros::param::get("~hand_info/origin", origin);
  ros::param::get("~hand_info/serial", serial);
  ros::param::get("~hand_info/version", version);

  // Initialize CAN device
  canDevice = 0;
  if(!sim) {
    canDevice = new allegro::AllegroHandDrv();
    if (canDevice->init()) {
        usleep(3000);
    }
    else {
        delete canDevice;
        canDevice = 0;
    }
  }

  // Start ROS time
  tstart = ros::Time::now();
  
  // Advertise current joint state publisher and subscribe to desired joint
  // states.
  joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
  joint_cmd_sub = nh.subscribe(DESIRED_STATE_TOPIC, 1, // queue size
                                &AllegroNode::desiredStateCallback, this);
 
  time_sub = nh.subscribe("timechange", 10, &AllegroNode::ControltimeCallback,this);
  force_sub = nh.subscribe("forcechange", 10, &AllegroNode::GraspforceCallback,this);
 
  marker_pub = nh.advertise<visualization_msgs::Marker>("fingertip_arrow_markers", 1);
}

AllegroNode::~AllegroNode() {
  if (canDevice) delete canDevice;
  delete mutex;
  nh.shutdown();
}

// Get Allegro Hand desired joint position
void AllegroNode::desiredStateCallback(const sensor_msgs::JointState &msg) {
  mutex->lock();
  desired_joint_state = msg;
  mutex->unlock();
}

// Get Allegro Hand motion control time from gui
void AllegroNode::ControltimeCallback(const std_msgs::Float32::ConstPtr& msg)
{
    motion_time = msg->data;
    pBHand->SetMotiontime(motion_time);
}

// Get Allegro Hand grasping force from gui
void AllegroNode::GraspforceCallback(const std_msgs::Float32::ConstPtr& msg)
{
    force_get = msg->data;
}

// Main publisher
void AllegroNode::publishData() {
  // current position, velocity and effort (torque), fingertip_sensor (Rviz) published
  current_joint_state.header.stamp = tnow;
  
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.position[i] = current_position[i]; /// current_position_filtered[i];
    current_joint_state.velocity[i] = current_velocity[i]; /// current_velocity_filtered[i];
    current_joint_state.effort[i] = desired_torque[i];


  }
  joint_state_pub.publish(current_joint_state);

  // fingertip_sensor_Rviz
  for (const auto& marker : markers) {
    marker_pub.publish(marker);
    ros::Duration(1e-6).sleep();
    }

}


void AllegroNode::Rviz_Arrow(){ 
  //  This is the function to visualize fingertip sensors on Rviz.

    tf2::Quaternion orientation;
    orientation.setRPY(0, -M_PI/4, 0); 

  for (int i = 0; i < 4; ++i) {
    markers[i].header.frame_id = frame_ids[i];
    markers[i].ns = namespaces[i];
    markers[i].type = visualization_msgs::Marker::ARROW;
    markers[i].action = visualization_msgs::Marker::ADD;
    markers[i].pose.orientation = tf2::toMsg(orientation);
    markers[i].scale.x = fingertip_sensor[i] / 5000.0;
    markers[i].scale.y = 0.005;
    markers[i].scale.z = 0.005;
    markers[i].color.r = 1.0;
    markers[i].color.a = 1.0;
  }
}

void AllegroNode::updateController() {

// TODO [Task 3]: Synchronize CAN device data with ROS joint states.
// 1. Check if 'canDevice->readCANFrames()' is negative; if so, call 'rclcpp::shutdown()'.
// 2. If 'canDevice->isJointInfoReady()', backup 'current_position' to 'previous_position'.
// 3. Compute velocity: '(current_position[i] - previous_position[i]) / dt'.
// 4. MANDATORY: Call 'canDevice->resetJointInfoReady()' at the end of the update block.
//END OF TODO
}

// Interrupt-based control is not recommended by Wonik Robotics. I have not tested it.
void AllegroNode::timerCallback(const ros::TimerEvent &event) {
  updateController();
}

ros::Timer AllegroNode::startTimerCallback() {
  ros::Timer timer = nh.createTimer(ros::Duration(0.001),
                                    &AllegroNode::timerCallback, this);
  return timer;
}
