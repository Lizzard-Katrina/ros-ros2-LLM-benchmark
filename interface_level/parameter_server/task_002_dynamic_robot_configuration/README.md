# Task 002: Dynamic Robot Configuration (Interface Level)

## 1. Brief Description
This task is derived from the core initialization logic of the classic `turtlesim` package. In ROS 1, the node typically reads background color parameters (`background_r/g/b`) from the global Parameter Server upon startup. 
When migrating to ROS 2, this logic is refactored into node-based local parameter management. This test case requires the LLM to implement the complete interface logic for declaring and retrieving background color configurations using `rclcpp::Node`.
--
source code:
```https://github.com/ros/ros_tutorials/blob/rolling/turtlesim/src/turtlesim.cpp```
## 2. Abstraction Strategy & Rationale
### Targeted Code Segment
We have abstracted the entire logic block responsible for parameter declaration and initialization within the `TurtleApp` constructor.

### Rationale
* **Paradigm Shift**: This task tests whether the LLM understands the ROS 2 **"Declare-before-use"** mechanism, rather than instinctively using the ROS 1 direct-read (getParam) pattern.
* **Data Structure Constraint**: By mandating the use of a single `std::vector<int64_t>` instead of three separate integers, we evaluate the LLM’s ability to adapt to **atomic configuration** patterns and complex data types in ROS 2.
* **Interface Strictness**: We check if the LLM correctly handles the `int64_t` type mapping, which is the standard integer width for the ROS 2 parameter interface layer.

## 3. Testcase Design & Expected Outcomes

The task is evaluated using 6 independent Oracle Testcases that utilize regex pattern matching to verify semantic and structural correctness:

| Testcase | Design Logic | Expected Outcome |
| :--- | :--- | :--- |
| `test_uses_ros2_declare_parameter` | Validates the use of the mandatory ROS 2 `declare_parameter` interface. | Code must contain `nh_->declare_parameter` or its variants. |
| `test_enforces_vector_int64_structure` | Verifies adherence to the `std::vector<int64_t>` data structure constraint. | Matches the container definition, ensuring no fallback to simple `int`. |
| `test_correct_parameter_naming` | Checks if the parameter key matches the required `background_color_rgb`. | String matching confirms the API call targets the correct key. |
| `test_default_value_integrity` | Verifies business logic consistency: checks for the correct default fallback color. | Successfully matches the specific numeric sequence `{69, 86, 255}`. |
| `test_no_ros1_nodehandle_residue` | Thoroughness check: ensures no legacy ROS 1 `getParam` or `ros::NodeHandle` remains. | Negative matching ensures the migrated code is free of obsolete keywords. |
| `test_type_strictness_casting` | Validates the linkage between declaration and variable assignment. | Matches an `auto` or `vector` assignment receiving the result of the declaration. |

---

## 4. Evaluation Criteria
* **Success (Pass)**: The generated code passes all Oracle matches and follows ROS 2 parameter lifecycle best practices.
* **Partial Fail**: The code attempts parameter retrieval but skips the mandatory `declare` step (e.g., using `get_parameter` without declaration).
* **Semantic Fail**: The parameter names or default values (RGB values) do not align with the original ROS 1 business logic or the provided constraints.
