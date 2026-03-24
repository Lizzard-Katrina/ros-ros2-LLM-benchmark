# Task 004: TurtleBot3 Dynamic Parameter Observer Migration

## 1. Brief Description
This task involves migrating the parameter handling logic of the `turtlebot3.cpp` node from ROS 1 to ROS 2. Unlike static parameter retrieval, the primary objective here is to implement an **Asynchronous Observer Pattern**. The developer must utilize a ROS 2 Asynchronous Parameter Client to monitor specific parameter events (`motors.profile_acceleration`) and execute a callback to synchronize these changes with hardware motor profiles, including necessary physical unit conversions.

---

## 2. Hole Design Strategy

The task utilizes two strategically placed "holes" to evaluate the model's depth of understanding regarding the ROS 2 parameter lifecycle:

### A. Static Initialization & Service Readiness (`init_dynamixel_sdk_wrapper`)
* **Design Intent**: To test if the model understands the service-based nature of ROS 2 parameters.
* **Key Concept**: In ROS 2, the parameter server is an asynchronous entity. The implementation must include `wait_for_service` logic to ensure the client connects to the server during node startup, preventing initialization failures if the service is not yet available.

### B. Asynchronous Event Response (`parameter_event_callback`)
* **Design Intent**: To evaluate the model's ability to parse complex message structures and maintain physical logic integrity.
* **Key Constraints**: 
    * **Architecture**: Models are strictly required to use the `on_parameter_event` stream rather than the simpler `add_on_set_parameters_callback` hook. This tests the ability to handle advanced asynchronous streams.
    * **Physics**: The migration must preserve the mathematical relationship: `Profile Acceleration = Value / Constant`. This is a critical indicator of "semantic migration" vs. "blind syntax translation."

---

## 3. Testcase Design & Expected Outcomes

The evaluation uses **Full-file Regex Scanning** to ensure the code meets both architectural and logical standards.

| Testcase | Design Principle | Expected Outcome |
| :--- | :--- | :--- |
| **`test_async_client_architecture`** | Verifies the use of the requested Asynchronous Observer architecture. | Successful detection of `AsyncParametersClient` instantiation. |
| **`test_service_readiness_logic`** | Checks for robust handling of ROS 2 asynchronous service characteristics. | Presence of `wait_for_service` logic with appropriate timeout. |
| **`test_api_constraint_compliance`** | Evaluates instruction following by excluding non-standard API alternatives. | Usage of `on_parameter_event`; absence of `add_on_set_parameters_callback`. |
| **`test_target_parameter_recognition`** | Confirms the logic targets the correct parameter string. | Match for the string `"motors.profile_acceleration"`. |
| **`test_physics_logic_preservation`** | **Core Logic Check**. Verifies the mathematical integrity of the migration. | Identification of the **Division (/)** operator between the value and the constant. |
| **`test_event_message_parsing`** | Validates understanding of the `ParameterEvent` message structure. | Iteration through the `event->changed_parameters` list. |
| **`test_value_extraction_style`** | Checks for standard ROS 2 parameter value extraction methods. | Usage of `.as_double()` or `from_parameter_msg` for type conversion. |
| **`test_logging_semantic_content`** | Verifies the feedback loop requirements. | Log message must contain the specific physical unit `"rev/min2"`. |
| **`test_no_legacy_ros1_symbols`** | Ensures complete migration and removal of legacy code. | Total absence of `ros::NodeHandle`, `getParam`, etc. (outside of comments). |

---

> **Note for Evaluators**: This task is a high-resolution benchmark. It successfully fails models that perform "syntax-only" translations (e.g., those that mistakenly use multiplication instead of division or bypass the requested Observer Pattern for simpler hooks).
