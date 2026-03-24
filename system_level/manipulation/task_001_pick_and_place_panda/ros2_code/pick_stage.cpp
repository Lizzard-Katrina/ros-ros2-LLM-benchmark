```cpp
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/approach.h>
#include <moveit/task_constructor/stages/attach_object.h>

void buildPickStage(moveit::task_constructor::Task& task) {
	using namespace moveit::task_constructor::stages;

	// Create the Pick stage, it includes grasp generator and attach + detach stages
	auto pick = std::make_unique<Pick>("pick");

	// Set the end-effector and the target object
	pick->setProperty("eef", "hand");
	pick->setProperty("target", "object");

	// Approach stage before pick (approach grasp)
	auto approach = std::make_unique<Approach>("approach");
	approach->setIKFrame("hand_grasp_frame");
	approach->properties().set("group", "arm");

	// Attach object after pick
	auto attach = std::make_unique<AttachObject>("attach");
	attach->setEndEffector("hand");
	attach->setObject("object");

	// Add stages to Pick: grasp generation + approach + attach
	pick->insert(std::move(approach));
	pick->insert(std::move(attach));

	task.add(std::move(pick));
}
```