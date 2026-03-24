# Task 005: Gazebo ROS Service Client Benchmark

## Brief Description

This task focuses on testing students' ability to implement **ROS2 service clients** that wrap Gazebo services.  
Specifically, three client functions from `gazebo_interface.py` are targeted:

- `spawn_sdf_model_client`
- `spawn_urdf_model_client`
- `set_model_configuration_client`

Students are expected to handle service calls correctly using `rclpy`, properly process responses, and handle exceptions.  
The goal is to ensure the client returns a meaningful success/failure value based on the actual service response rather than hard-coded literals.


---

## Source Code Overview & Logic Removed

`https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros/gazebo_interface.py#L34`
### Removal 1: `try/except` blocks

The `try/except` blocks around the service calls were removed.  

**Reason:** To force students (or an LLM) to explicitly implement exception handling logic.  
The task requires them to catch `rclpy.ServiceException` (or equivalent) and return `False` on failure.

---

### Removal 2: Response handling (`resp.success`)

The code that directly reads the service response (`resp.success`) and returns it was removed.  

**Reason:** Students must implement the logic that extracts the success/failure information from the service response.  
This prevents trivial hardcoded returns like `return True` or `return False`.

---

### Removal 3: Logging of service calls

`rospy.loginfo()` statements for the service call and its status message were removed.  

**Reason:** To encourage students to include meaningful logging themselves, and to focus on the functional logic rather than copied boilerplate.

---

### Removal 4: Sleep / race-condition workaround

In `set_model_configuration_client`, the `time.sleep(1)` hack to fix race conditions was removed.  

**Reason:** The task focuses on implementing the core service client logic and handling responses correctly.  
Students may optionally reintroduce timing considerations if needed.

---
## Oracle Test

### Test 1: `test_return_depends_on_service_response`

**Purpose:** Ensure the return value depends on the service response, not a constant `True/False`.  

**Expected Outcome:** The student’s code must use `response.success` (or equivalent) to determine the return value.  
Hardcoded `return True` or `return False` will fail this test.

### Test 2: `test_failure_path_exists`

**Purpose:** Verify there is at least one failure path that results in returning `False`.  

**Expected Outcome:** The code must handle service failures or exceptions, mapping them to a `False` return.  
This ensures robust handling of unsuccessful service calls.
### Test 3: `test_response_used_in_return`

**Purpose:** Ensure the service response object is actually used in determining the return value.  

**Expected Outcome:** The `response` object must appear in the return statement or in intermediate variables that influence the return.  
Trivial or unused service calls will fail this test.
### Test 4: `test_no_trivial_success_assignment`

**Purpose:** Ensure the `success` variable is derived from the service response or other runtime checks, not hardcoded.  

**Expected Outcome:** Assignments like `success = True` or `success = False` are forbidden.  
The return value must be dynamically computed from the service call.
### Test 5: `test_exception_handled`

**Purpose:** Ensure that exceptions during service calls are caught.  

**Expected Outcome:** The code must include a `try/except` block (or equivalent) around the service call.  
Failing to handle exceptions will cause this test to fail.
### Test 6: `test_multiple_service_calls_use_response`

**Purpose:** Verify that if multiple service calls exist (`spawn_sdf_model_client`, `spawn_urdf_model_client`, `set_model_configuration_client`), each response is used appropriately.  

**Expected Outcome:** Each service call’s `response` object must be referenced in the return logic or intermediate success computations.  
Ignoring a response or not using it will fail this test.
### Test 7: `test_syntax_check`

**Purpose:** Check that the code is syntactically valid Python and can be parsed/compiled.  

**Expected Outcome:** No syntax errors; the code should import required modules and define the service client functions.  
This prevents submission of incomplete or malformed code.

