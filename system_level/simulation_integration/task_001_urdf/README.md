# Benchmark Task: System-Level Simulation Integration (URDF to ROS 2)

## 1. Brief Description
This task evaluates an AI's ability to perform a **cross-format architectural migration** and maintain **system-level consistency**. The input consists of a legacy ROS 1 robot description package, including an XML-ive `.launch` file and a `.urdf.xacro` file with specific hollowing (holes). 

The core challenge is not just "filling in the blanks," but **reconstructing the functional bridge** between the robot's physical description and the ROS 2 simulation pipeline. The model must recognize that a ROS 1 XML launch is deprecated and must be rewritten as a Python `launch.py` script to support dynamic Xacro processing and ROS 2 parameter conventions.

---
source code file:
```


## 2. Hollowing Design Logic

### A. `08-macroed.urdf.xacro` (Structural & Physics Consistency)
* **Hole Location:** The `base_link` definition (visual, collision, and inertial tags).
* **Design Intent:**
    * **Property Mapping:** Verifies if the model correctly utilizes pre-defined Xacro properties `${width}` and `${bodylen}` instead of hardcoding arbitrary values.
    * **Physics Awareness:** In a simulation context, missing or incorrect `<inertial>` blocks cause the robot to fail in Gazebo (e.g., vanishing or flying away). This tests the model's understanding of "Simulation-Ready" URDFs.
    * **Instruction Adherence:** By requiring a `cylinder` specifically, we test if the model follows technical constraints or simply defaults to a generic `box`.

### B. `display.launch.py` (API & Framework Migration)
* **Hole Location:** The core logic for package path resolution and node parameter assignment.
* **Design Intent:**
    * **Format Migration:** This is the primary "System-Level" hurdle. The model must transition from XML tags to a Python `LaunchDescription`.
    * **Dynamic Command Execution:** ROS 2 requires the `xacro` command to be executed at runtime. The model must use the `launch.substitutions.Command` API.
    * **Environment Resolution:** Tests the migration from `$(find)` (ROS 1) to `get_package_share_directory` (ROS 2/Ament) or `FindPackageShare`.

---

## 3. Oracle Testcase Design & Expected Outcomes

The Oracle validates the solution using independent **pytest** cases that perform semantic pattern matching (regex).

| Testcase Name | Semantic Concept | Expected Outcome for Pass |
| :--- | :--- | :--- |
| `test_urdf_base_link_consistency` | **Property Binding** | Must contain `<cylinder>` using `${width}` and `${bodylen}` properties. |
| `test_urdf_inertial_integration` | **Simulation Physics** | Must call `<xacro:default_inertial mass="10" />` within the link scope. |
| `test_launch_xacro_command_api` | **Runtime Execution** | Must use `Command(['xacro', ...])` to process the URDF into a string. |
| `test_launch_resource_indexing` | **Ament Environment** | Must use `get_package_share_directory` or `FindPackageShare`. |
| `test_launch_parameter_wrapping` | **Data Encapsulation** | `robot_description` must be wrapped in `ParameterValue` for XML string safety. |
| `test_anti_leakage_ros1` | **Legacy Cleanup** | No instances of `$(find ...)` or `$(arg ...)` should remain. |
| `test_launch_integration` | **System Completeness** | Must correctly include the external `display.launch.py` from `urdf_launch`. |

---

## 4. Evaluation Criteria (Success vs. Failure)

* **Pass (Expert Level):** The model produces a `.launch.py` Python script, maps all ROS 1 macros to ROS 2 substitutions, and maintains exact physical dimensions in the URDF.
* **Partial Fail (Hallucination):** The model fills the URDF but uses the wrong mass (e.g., `20` instead of `10`) or the wrong shape (e.g., `box` instead of `cylinder`).
* **Critical Fail (Lazy Completion):** The model keeps the ROS 1 XML format and simply adds ROS 2-style node tags inside the XML. This indicates a failure to understand the fundamental architectural shift between ROS versions.
