```c++
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <rclcpp/rclcpp.hpp>

void executeTask(moveit::task_constructor::Task& task) {
	rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("moveit_task_executor");

	if (!task.plan()) {
		RCLCPP_ERROR(node->get_logger(), "Planning failed");
		return;
	}

	if (!task.execute()) {
		RCLCPP_ERROR(node->get_logger(), "Execution failed");
		return;
	}

	RCLCPP_INFO(node->get_logger(), "Task executed successfully");
}
```
