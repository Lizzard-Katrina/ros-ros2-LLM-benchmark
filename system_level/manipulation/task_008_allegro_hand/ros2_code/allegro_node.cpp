// Common allegro node code used by any node. Each node that implements an
// AllegroNode must define the computeDesiredTorque() method.
//
// Editor: Hibo (sh-yang @ wonik.com)

#include "allegro_node.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

std::string jointNames[DOF_JOINTS] =
        {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
                "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",
                "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0",
        };

std::vector<std::string> frame_ids = {"link_3_0_tip", "link_7_0_tip", "link_11_0_tip", "link_15_0_tip"};
std::vector<std::string> namespaces = {"fingertip_[0]", "fingertip_[1]", "fingertip_[2]", "fingertip_[3]"};
std::vector<visualization_msgs::msg::Marker> markers(4);

AllegroNode::AllegroNode(bool sim /* = false */) : Node("allegro_node") {
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
  std::string robot_name, manufacturer, origin, serial;
  double version;
  this->declare_parameter("hand_info.robot_name", "");
  this->declare_parameter("hand_info.which_hand", "");
  this->declare_parameter("hand_info.which_type", "");
  this->declare_parameter("hand_info.manufacturer", "");
  this->declare_parameter("hand_info.origin", "");
  this->declare_parameter("hand_info.serial", "");
  this->declare_parameter("hand_info.version", 0.0);

  this->get_parameter("hand_info.robot_name", robot_name);
  this->get_parameter("hand_info.which_hand", whichHand);
  this->get_parameter("hand_info.which_type", whichType);
  this->get_parameter("hand_info.manufacturer", manufacturer);
  this->get_parameter("hand_info.origin", origin);
  this->get_parameter("hand_info.serial", serial);
  this->get_parameter("hand_info.version", version);

  // Initialize CAN device
  canDevice = 0;
  if(!sim) {
    canDevice = new allegro::AllegroHandDrv();
    if (canDevice->init(0)) {
        usleep(3000);
    }
    else {
        delete canDevice;
        canDevice = 0;
    }
  }

  // Start ROS time
  tstart = this->now();
  
  // Advertise current joint state publisher and subscribe to desired joint
  // states.
  joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(JOINT_STATE_TOPIC, 3);
  joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      DESIRED_STATE_TOPIC, 1, std::bind(&AllegroNode::desiredStateCallback, this, std::placeholders::_1));
 
  time_sub = this->create_subscription<std_msgs::msg::Float32>(
      "timechange", 10, std::bind(&AllegroNode::ControltimeCallback, this, std::placeholders::_1));
  force_sub = this->create_subscription<std_msgs::msg::Float32>(
      "forcechange", 10, std::bind(&AllegroNode::GraspforceCallback, this, std::placeholders::_1));
 
  marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("fingertip_arrow_markers", 1);
}

AllegroNode::~AllegroNode() {
  if (canDevice) delete canDevice;
  delete mutex;
}

// Get Allegro Hand desired joint position
void AllegroNode::desiredStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  mutex->lock();
  desired_joint_state = *msg;
  mutex->unlock();
}

// Get Allegro Hand motion control time from gui
void AllegroNode::ControltimeCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    motion_time = msg->data;
    if (pBHand) pBHand->SetMotiontime(motion_time);
}

// Get Allegro Hand grasping force from gui
void AllegroNode::GraspforceCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    force_get = msg->data;
}

// Main publisher
void AllegroNode::publishData() {
  // current position, velocity and effort (torque), fingertip_sensor (Rviz) published
  tnow = this->now();
  current_joint_state.header.stamp = tnow;
  
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.position[i] = current_position[i]; /// current_position_filtered[i];
    current_joint_state.velocity[i] = current_velocity[i]; /// current_velocity_filtered[i];
    current_joint_state.effort[i] = desired_torque[i];
  }
  joint_state_pub->publish(current_joint_state);

  // fingertip_sensor_Rviz
  for (const auto& marker : markers) {
    marker_pub->publish(marker);
  }
}


void AllegroNode::Rviz_Arrow(){ 
  //  This is the function to visualize fingertip sensors on Rviz.

    tf2::Quaternion orientation;
    orientation.setRPY(0, -M_PI/4, 0); 

  for (int i = 0; i < 4; ++i) {
    markers[i].header.frame_id = frame_ids[i];
    markers[i].ns = namespaces[i];
    markers[i].type = visualization_msgs::msg::Marker::ARROW;
    markers[i].action = visualization_msgs::msg::Marker::ADD;
    markers[i].pose.orientation = tf2::toMsg(orientation);
    markers[i].scale.x = fingertip_sensor[i] / 5000.0;
    markers[i].scale.y = 0.005;
    markers[i].scale.z = 0.005;
    markers[i].color.r = 1.0;
    markers[i].color.a = 1.0;
  }
}

void AllegroNode::updateController() {
    if (canDevice) {
        if (canDevice->readCANFrames() < 0) {
            rclcpp::shutdown();
            return;
        }
        
        if (canDevice->isJointInfoReady()) {
            for (int i = 0; i < DOF_JOINTS; i++) {
                previous_position[i] = current_position[i];
            }
            
            canDevice->getJointInfo(current_position);
            
            double dt = 0.003; // Assuming 3ms loop time
            for (int i = 0; i < DOF_JOINTS; i++) {
                current_velocity[i] = (current_position[i] - previous_position[i]) / dt;
            }
            
            canDevice->resetJointInfoReady();
        }
    }
}

void AllegroNode::timerCallback() {
  updateController();
}

rclcpp::TimerBase::SharedPtr AllegroNode::startTimerCallback() {
  return this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&AllegroNode::timerCallback, this));
}