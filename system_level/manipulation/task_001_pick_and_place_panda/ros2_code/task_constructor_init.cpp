```cpp
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

static rclcpp::Node::SharedPtr node;
static moveit::core::RobotModelPtr robot_model;

// Initialize the MoveIt Task Constructor pipeline
// Initialize ROS2 node and load robot model once
void initializePipeline()
{
	rclcpp::init(0, nullptr);
	node = rclcpp::Node::make_shared("moveit_task_constructor_node");
	robot_model_loader::RobotModelLoader rm_loader(node, "robot_description");
	robot_model = rm_loader.getModel();
}

void initTask(moveit::task_constructor::Task& task) {
	// initialize task properties and robot model
	if (!node) {
		initializePipeline();
	}
	task.setRobotModel(robot_model);
	task.setName("MoveItTaskConstructorTaskROS2");
	task.setNode(node);
}
```