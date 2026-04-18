# Benchmark Task 007: AWS Lex Node System-Level Migration

## 1. Brief Description
This task involves migrating the core communication node of the `aws-lex-ros1` package to **ROS 2 (Humble/Iron)**. The `LexNode` acts as a bridge between ROS service calls and the AWS Lex SDK. 

At the **System Level**, this task evaluates the LLM's ability to maintain **Static Architectural Consistency**. Unlike file-level tasks, the model must synchronize the transition from a "Composition" pattern (holding a `ros::NodeHandle`) to an "Inheritance" pattern (`rclcpp::Node`), ensuring that service declarations in the header exactly match the instantiation and callback logic in the source file.

---

source code file:
```https://github.com/aws-robotics/lex-ros1/blob/master```

## 2. Hollowing Strategy & Interdependence

To test system-level intelligence, the holes are placed to break the functional link between the interface (`.h`) and the implementation (`.cpp`).

### **Hole A: Header Infrastructure & Callbacks (`lex_node.h`)**
* **Hole:** Private members (`lex_server_`) and the `LexServerCallback` signature.
* **Reason:** Forces the model to define the ROS 2 type system (e.g., `rclcpp::Service::SharedPtr`). 
* **Coupling:** If the model chooses a specific variable name or shared pointer type here, it **must** use the exact same name/type in the `.cpp` file, or the system will fail to link/compile.

### **Hole B: The Constructor (`lex_node.cpp`)**
* **Hole:** The entire `LexNode::LexNode()` body.
* **Reason:** In ROS 2, parameter declaration (`declare_parameter`) is a prerequisite for parameter usage. 
* **Coupling:** This tests if the model recognizes that configuration logic previously handled in `Init` or implicitly by `NodeHandle` now belongs in the constructor phase.

### **Hole C: Initialization Logic (`lex_node.cpp`)**
* **Hole:** The entire `ErrorCode LexNode::Init(...)` function.
* **Reason:** This is the "glue" code. It must connect the member variables declared in the header to the ROS 2 `create_service` factory method.
* **Coupling:** It requires the model to correctly implement `std::bind` referencing the class method declared in the header, testing the model's "memory" across file boundaries.

---

## 3. Oracle Testcase Design & Expected Outcomes

The Oracle uses regex to validate **concepts and consistency** rather than just line-by-line matching.

| Testcase | Design Logic | Expected Outcome (Pass Criteria) |
| :--- | :--- | :--- |
| **Member Consistency** | Ensures the variable name for the service server is identical in both files. | A declaration `SharedPtr lex_server_` in `.h` and an assignment `lex_server_ = ...` in `.cpp`. |
| **Callback Sync** | Checks if the `SharedPtr` signature for Request/Response was applied to both the declaration and implementation. | Both files must use `(std::shared_ptr<...Request>, std::shared_ptr<...Response>)` or equivalent. |
| **Parameter Pattern** | Validates the "Declare-before-Use" best practice in ROS 2. | `this->declare_parameter` must be found inside the constructor body, not just the `Init` function. |
| **Service Binding** | Enforces the use of `std::bind` with class methods to check for strict API adherence. | Presence of `std::bind(&LexNode::LexServerCallback, this, _1, _2)` or similar placeholder logic. |
| **Inheritance Pattern** | Verifies the structural shift to the ROS 2 standard Node class. | The class definition must include `: public rclcpp::Node`. |
| **Namespace Migration** | Targets the specific C++ namespace changes for ROS 2 services. | Replacement of ROS 1 message types with the `::srv::` nested namespace. |
| **Anti-Leakage** | Scans for "lazy" migration where ROS 1 artifacts are left behind. | **Zero** occurrences of `ros::NodeHandle`, `advertiseService`, or ROS 1 header includes. |
| **Defensive Coding** | Ensures that during the heavy refactor, the model didn't lose the original safety checks. | The `if (!post_content)` null-check must remain present in the migrated `Init` function. |

