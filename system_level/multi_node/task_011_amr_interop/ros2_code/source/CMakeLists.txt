cmake_minimum_required(VERSION 3.8)
project(task_011_amr_interop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionParameter.msg"
  "msg/Action.msg"
  "msg/NodePosition.msg"
  "msg/ControlPoint.msg"
  "msg/Trajectory.msg"
  "msg/Node.msg"
  "msg/Edge.msg"
  "msg/Order.msg"
  "msg/InstantActions.msg"
  "msg/Connection.msg"
  "msg/AGVPosition.msg"
  "msg/Velocity.msg"
  "msg/BatteryState.msg"
  "msg/Error.msg"
  "msg/ErrorReference.msg"
  "msg/Info.msg"
  "msg/InfoReference.msg"
  "msg/SafetyState.msg"
  "msg/ActionState.msg"
  "msg/EdgeState.msg"
  "msg/NodeState.msg"
  "msg/Load.msg"
  "msg/OrderState.msg"
  "msg/Visualization.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python modules
ament_python_install_package(vda5050_connector_py)

# Install Python scripts
install(PROGRAMS
  scripts/mqtt_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()