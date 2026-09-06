// Common allegro node code used by any node. Each node that implements an
// AllegroNode must define the computeDesiredTorque() method.
//
// Ported to ROS2 Humble

#include "allegro_node.h"

std::string jointNames[DOF_JOINTS] =
        {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
                "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",
                "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0",
        };

AllegroNode::AllegroNode(bool sim /* = true */)
  : Node("allegro_node")
{
  mutex = new std::mutex();

  // Create arrays 16 long for each of the four joint state components
  current_joint_state.position.resize(DOF_JOINTS);
  current_joint_state.velocity.resize(DOF_JOINTS);
  current_joint_state.effort.resize(DOF_JOINTS);
  current_joint_state.name.resize(DOF_JOINTS);

  // Initialize values
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.name[i] = jointNames[i];
    desired_torque[i] = 0.0;
    current_velocity[i] = 0.0;
    current_position_filtered[i] = 0.0;
    current_velocity_filtered[i] = 0.0;
  }

  // Initialize CAN device
  canDevice = nullptr;
  if(!sim) {
    canDevice = new allegro::AllegroHandDrv();
    if (canDevice->init()) {
        usleep(3000);
    }
    else {
        delete canDevice;
        canDevice = nullptr;
    }
  } else {
    // In sim mode, create the device and enable sim mode
    canDevice = new allegro::AllegroHandDrv();
    canDevice->setSimMode(true);
  }

  // Start ROS time
  tstart = this->now();

  // Advertise current joint state publisher and subscribe to desired joint states.
  joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(JOINT_STATE_TOPIC, 3);
  joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      DESIRED_STATE_TOPIC, 1,
      std::bind(&AllegroNode::desiredStateCallback, this, std::placeholders::_1));
}

AllegroNode::~AllegroNode() {
  if (canDevice) delete canDevice;
  delete mutex;
}

void AllegroNode::desiredStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  mutex->lock();
  desired_joint_state = *msg;
  mutex->unlock();
}

void AllegroNode::publishData() {
  current_joint_state.header.stamp = tnow;

  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.position[i] = current_position[i];
    current_joint_state.velocity[i] = current_velocity[i];
    current_joint_state.effort[i] = desired_torque[i];
  }
  joint_state_pub->publish(current_joint_state);
}

void AllegroNode::updateController() {

  // Calculate loop time;
  tnow = this->now();
  dt = ALLEGRO_CONTROL_TIME_INTERVAL;

  if(dt <= 0) {
    return;
  }

  tstart = tnow;

  if (canDevice)
  {
    // try to update joint positions through CAN comm:
    lEmergencyStop = canDevice->readCANFrames();

    // Safety check: if driver reports error, shutdown immediately
    if (lEmergencyStop < 0) {
      rclcpp::shutdown();
      return;
    }

    // check if all positions are updated:
    if (canDevice->isJointInfoReady())
    {
      // back-up previous joint positions:
      for (int i = 0; i < DOF_JOINTS; i++) {
        previous_position[i] = current_position[i];
        previous_position_filtered[i] = current_position_filtered[i];
        previous_velocity[i] = current_velocity[i];
      }

      // update joint positions:
      canDevice->getJointInfo(current_position);

      // compute velocity using finite difference:
      for (int i = 0; i < DOF_JOINTS; i++) {
        current_position_filtered[i] = current_position[i];
        current_velocity[i] = (current_position[i] - previous_position[i]) / dt;
        current_velocity_filtered[i] = current_velocity[i];
      }

      // compute desired torque (virtual, implemented by subclass):
      computeDesiredTorque();

      // write torques to hardware:
      if (canDevice) {
        canDevice->setTorque(desired_torque);
        canDevice->writeJointTorque();
      }

      // publish data:
      publishData();

      frame++;

      // MANDATORY: Reset the joint info ready bitmask for next cycle
      canDevice->resetJointInfoReady();
    }
  }
}

void AllegroNode::timerCallback() {
  updateController();
}

rclcpp::TimerBase::SharedPtr AllegroNode::startTimerCallback() {
  auto timer = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&AllegroNode::timerCallback, this));
  return timer;
}

void AllegroNode::injectTestPositions(double *positions) {
  if (canDevice) {
    canDevice->injectJointPositions(positions);
    canDevice->setAllJointsReady();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<AllegroNode>(true);

  // Inject some test data so the node publishes something
  double test_pos[DOF_JOINTS];
  for (int i = 0; i < DOF_JOINTS; i++) {
    test_pos[i] = 0.1 * (i + 1);
  }
  node->injectTestPositions(test_pos);

  auto timer = node->startTimerCallback();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}