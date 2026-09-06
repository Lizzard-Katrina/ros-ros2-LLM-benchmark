#ifndef PROJECT_ALLEGRO_NODE_COMMON_H
#define PROJECT_ALLEGRO_NODE_COMMON_H

#include "allegro_hand_driver/AllegroHandDrv.h"
using namespace allegro;

#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

// Set control time interval
#define ALLEGRO_CONTROL_TIME_INTERVAL 0.002

// Topic names
const std::string JOINT_STATE_TOPIC = "allegroHand/joint_states";
const std::string DESIRED_STATE_TOPIC = "allegroHand/joint_cmd";
const std::string LIB_CMD_TOPIC = "allegroHand/lib_cmd";


class AllegroNode : public rclcpp::Node {
 public:

  AllegroNode(bool sim = true);

  virtual ~AllegroNode();

  void publishData();

  void desiredStateCallback(const sensor_msgs::msg::JointState::SharedPtr desired);

  virtual void updateController();

  virtual void computeDesiredTorque() {
    RCLCPP_ERROR(this->get_logger(), "Called virtual function!");
  };

  rclcpp::TimerBase::SharedPtr startTimerCallback();

  void timerCallback();

  // Public method to inject test positions into the CAN device (for testing)
  void injectTestPositions(double *positions);

 protected:

  double current_position[DOF_JOINTS] = {0.0};
  double previous_position[DOF_JOINTS] = {0.0};

  double current_position_filtered[DOF_JOINTS] = {0.0};
  double previous_position_filtered[DOF_JOINTS] = {0.0};

  double current_velocity[DOF_JOINTS] = {0.0};
  double previous_velocity[DOF_JOINTS] = {0.0};
  double current_velocity_filtered[DOF_JOINTS] = {0.0};

  double desired_torque[DOF_JOINTS] = {0.0};

  std::string whichHand;
  std::string whichType;

  // ROS2 publishers/subscribers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub;

  // Store the current and desired joint states.
  sensor_msgs::msg::JointState current_joint_state;
  sensor_msgs::msg::JointState desired_joint_state;

  // Time
  rclcpp::Time tstart;
  rclcpp::Time tnow;
  double dt;

  // CAN device
  allegro::AllegroHandDrv *canDevice;
  std::mutex *mutex;

  // Flags
  int lEmergencyStop = 0;
  long frame = 0;
};

#endif //PROJECT_ALLEGRO_NODE_COMMON_H