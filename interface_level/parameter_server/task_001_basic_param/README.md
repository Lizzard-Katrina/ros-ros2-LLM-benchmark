# Task 001 – Basic Parameter Server

This task evaluates the LLM's ability to translate ROS1 parameter server operations into ROS2 equivalents.

The task uses three single-hole ROS1 code files, each testing one transformation skill.

---

## Files and TODO Holes

### **version_01_read_param.cpp**
- **TODO_1:** Load `robot_name` and `start_mode` from the parameter server.
- Focus: ROS1 `getParam` → ROS2 `get_parameter`.

### **version_02_log_param.cpp**
- **TODO_2:** Log retrieved parameters.
- Focus: ROS1 `ROS_INFO` → ROS2 `RCLCPP_INFO`.

### **version_03_update_param.cpp**
- **TODO_3:** Update a parameter at runtime.
- Focus: ROS1 `setParam` → ROS2 `set_parameters`.

Each hole is isolated so evaluation can measure specific sub-skill performance.

---

## Expected Outcomes

### version_01_read_param.cpp
Must correctly load parameters from: config/default_params.yaml

### version_02_log_param.cpp
Must print something like:
robot_name: PR2
start_mode: 1

### version_03_update_param.cpp
Should print:
Original start_mode: 1
Updated start_mode: <new value>

---

## Docker Usage

### Build the image
```bash
cd docker
docker build -t param_task_001 .```
### Run container
docker run -it --name param001 param_task_001
Inside container: set environment
source /opt/ros/noetic/setup.sh
source /ros1_ws/devel/setup.sh
source /opt/ros/foxy/setup.sh
source /ros2_ws/install/setup.sh

### Run each test
rosrun task_001_basic_param version_01_read_param
rosrun task_001_basic_param version_02_log_param
rosrun task_001_basic_param version_03_update_param


---

# 7️⃣ **expected_ros2/expected_conversion_notes.md**

```markdown
# Expected ROS2 Conversions

- Parameters must be declared using `declare_parameter()`.
- Parameter retrieval must use `get_parameter("name").as_<type>()`.
- Logging must use `RCLCPP_INFO(node->get_logger(), ...)`.
- Runtime parameter updates must use `set_parameters()`.
