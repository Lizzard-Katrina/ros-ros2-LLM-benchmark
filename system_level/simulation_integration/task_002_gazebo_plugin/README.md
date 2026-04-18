# Benchmark Task: Gazebo Plugin & Launch Integration

## 1. Brief Description
This task evaluates the model's ability to implement a **Gazebo World Plugin** and integrate it into a **ROS 1 XML Launch system**. It tests two distinct domains: 
1. **C++ Simulation Logic**: Correctly implementing the Gazebo Plugin lifecycle.
2. **Launch Modularity**: Reusing existing launch files (`empty_world.launch`) and overriding parameters with package-specific paths.
---
source code
```https://github.com/ros-simulation/gazebo_ros_demos/blob/kinetic-devel/gazebo_tutorials```


## 2. Hollowing Design Logic

### A. `simple_world_plugin.cpp` (Plugin Lifecycle)
* **Hole Location**: The `Load` function implementation.
* **Design Intent**: 
    * **Lifecycle Awareness**: In Gazebo, the constructor is often too early for world interaction. The `Load` function is the mandatory entry point where the `physics::WorldPtr` and `sdf::ElementPtr` are provided. 
    * **Pointer Handling**: Tests if the model understands how Gazebo injects pointers into the plugin.
    * **Trap**: Providing a constructor with ROS initialization code often "tricks" models into forgetting the `Load` function entirely.

### B. `hello.launch` (XML Launch Reusability)
* **Hole Location**: The `<include>` block for Gazebo's empty world.
* **Design Intent**:
    * **Inheritance Pattern**: Tests if the model knows how to wrap `gazebo_ros/launch/empty_world.launch` instead of trying to launch Gazebo from scratch.
    * **Path Precision**: Verifies the use of `$(find gazebo_tutorials)` to locate the custom `.world` file.
    * **Argument Mapping**: Tests if the model correctly identifies the specific argument name (`world_name`) used by the standard Gazebo launch scripts.

---

## 3. Oracle Testcase Design & Expected Outcomes

The Oracle uses **pytest** with regex to enforce technical accuracy and prevent "Training Data Hallucination" (e.g., the model trying to use `husky_gazebo` instead of the task's package).

| Testcase Name | Semantic Concept | Expected Outcome for Pass |
| :--- | :--- | :--- |
| `test_cpp_load_function_signature` | **Gazebo API Compliance** | Must implement `void Load(physics::WorldPtr ..., sdf::ElementPtr ...)` exactly. |
| `test_cpp_plugin_registration` | **Plugin Export** | Must contain the `GZ_REGISTER_WORLD_PLUGIN` macro at the end of the file. |
| `test_launch_empty_world_inclusion` | **Dependency Injection** | Must include `$(find gazebo_ros)/launch/empty_world.launch`. |
| `test_launch_world_parameter_passing` | **Path Accuracy** | Must pass the argument `world_name` pointing specifically to `gazebo_tutorials/worlds/hello.world`. |
| `test_anti_leakage_ros2` | **Framework Isolation** | Must NOT use ROS 2 keywords like `LaunchDescription` or `Node(` in this ROS 1 task. |

---

## 4. Why Models Fail This Task
* **API Neglect**: Models often put logic in the constructor and skip the `Load` function, which is a functional failure in Gazebo.
* **Contextual Hallucination**: Models frequently replace `gazebo_tutorials` with `husky_gazebo` or `turtlebot3_gazebo` because those strings appeared more often in their training data.
* **Argument Confusion**: Models often guess the argument name as `world` instead of the standard `world_name`.
