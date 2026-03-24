# Task: ROS 1 to ROS 2 Parameter Server Migration

## 1. Brief Description
This task focuses on migrating the parameter handling logic of a `diff_drive_controller` from **ROS 1 (cpp)** to **ROS 2 (rclcpp)**. 

The goal is to evaluate whether an LLM can not only replace legacy APIs (e.g., `ros::NodeHandle::param`) with modern equivalents (`rclcpp::Node::declare_parameter`) but also adhere to **strict style constraints** and **logic paradigms** (such as exception handling and specific logging macros) defined in the source code comments.

---
source code:
```https://github.com/ros-controls/ros_controllers/blob/noetic-devel/diff_drive_controller/src/diff_drive_controller.cpp#L166```
## 2. Motivation & Logic for Each "Hole"

### Hole 1 & 2: Controller Initialization & Fallback Logic
* **Location**: Inside the `init()` function.
* **Reasoning**: 
    * **API Paradigm Shift**: In ROS 1, parameters are often retrieved into existing variables. In ROS 2, parameters must be **declared** before use.
    * **Logic Branching**: The task requires handling a "fallback" scenario: if a global multiplier (`wheel_radius_multiplier`) is missing, the code must independently fetch left and right multipliers. This tests the LLM's ability to use `has_parameter()` or `get_parameter_or()` correctly.
    * **Constraint Testing**: We explicitly mandate the use of `RCLCPP_INFO_STREAM` to see if the LLM defaults to its training bias (`RCLCPP_INFO`) or follows the prompt's specific requirement.

### Hole 3: Complex Type Parsing (Covariance Diagonals)
* **Location**: Inside `setOdomPubFields()`.
* **Reasoning**:
    * **Data Structure Mapping**: ROS 1 uses `XmlRpc::XmlRpcValue` to parse arrays, which is verbose and unsafe. ROS 2 supports `std::vector<T>` natively.
    * **Validation & Error Handling**: The task enforces a strict size check (must be 6 elements) and requires throwing a `std::invalid_argument`. This evaluates if the LLM can implement robust C++ error-handling patterns instead of just logging a warning.

---

## 3. Oracle Test Design & Expected Outcomes

The Oracle tests use **Static Pattern Matching (Regex)** to verify semantic and stylistic compliance.

| Oracle Test Name | Design Intent / Logic | Expected Outcome (Success Criteria) |
| :--- | :--- | :--- |
| `test_ros2_logging_style_strict` | Verifies adherence to the requested logging macro. | Must contain `RCLCPP_INFO_STREAM`. Using `RCLCPP_INFO` triggers a **Fail**. |
| `test_param_assignment_structure` | Ensures parameters are properly declared and stored in class members. | Must see `member_variable_ = node->declare_parameter...`. |
| `test_exception_message_strict` | Tests precision in error reporting. | Must throw an exception with the **exact** string `"diagonal size must be 6"`. Any prefix or suffix triggers a **Fail**. |
| `test_covariance_vector_type` | Validates the use of modern C++ containers for parameters. | Must use `std::vector<double>` in the template of `declare_parameter`. |
| `test_size_logic_check` | Checks if the required safety validation exists. | Must contain a logic branch checking if `.size() != 6`. |
| `test_no_ros1_remnants` | Ensures no "leaked" ROS 1 code remains in the solution. | Searches for `XmlRpc`, `ros::NodeHandle`, etc. Any hit triggers a **Fail**. |

---

## 4. Evaluation Criteria
* **Logical Correctness (50%)**: Does the code perform the correct parameter fetch and validation based on ROS 2 standards?
* **Instruction Following (50%)**: Does the code respect the **exact** string requirements and **specific** macro choices (`_STREAM`) defined in the TODO? 

> **Note**: A "Partial Pass" is often observed where the logic is correct but the model fails the strict style tests due to model-specific training biases.
