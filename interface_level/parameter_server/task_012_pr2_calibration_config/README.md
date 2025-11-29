# Task 012 — PR2 Robot Calibration Config

## Task Description
This benchmark task examines how an LLM migrates ROS1 parameter loading used for PR2 calibration to ROS2.  
The node reads robot calibration parameters (arm offsets, gripper zero offsets, joint-level calibration values) from the ROS1 parameter server and prints them.  
**Only the parameter loading portion is intentionally left blank** (TODO).  
Other calibration logic and logging remain unchanged for isolated evaluation of parameter migration.


## Hole Description

### version01_calibration_params.py

| Hole ID | Task | Expected ROS1 Behavior | Expected ROS2 Migration |
|--------|------|------------------------|--------------------------|
| TODO (only hole) | Load multiple calibration parameters from ROS1 parameter server | Uses `rospy.get_param` or `ros::param::get()` to retrieve calibration params (`arm_offset`, `gripper_zero`, `base_pitch_offset`, etc.) | Convert to ROS2 parameters: declare them at node startup (`declare_parameter()`), then retrieve via `get_parameter()` |

**Important:** The model **should not** fill in values (e.g., `"0.1"`) but replace the logic with correct ROS2 API calls.

## Example Calibration Parameters (for context)

These appear in ROS1 launch/EPR YAML files (not part of required solution):


pr2:
calibration:
arm_offset: 1.5
gripper_zero: 0.02
joints:
base_pitch: 0.005
shoulder: 0.002


**Outcome expectations** after migration:

✅ ROS2 nodes will declare calibration parameters (via `declare_parameter`)  
✅ Parameters retrieved via `get_parameter` and used for calibration logic  
✅ Calibration behavior remains consistent with ROS1 functional behavior

## Docker Usage (可先忽略，后续验证阶段再运行)

### Build the Docker image:

```bash
cd docker
chmod +x setup.sh
./setup.sh
```
### Run
```
docker run -it --rm pr2_calib_params /bin/bash
```

### Useful inside container:
```
rosrun task_012_pr2_calibration_config version01_calibration_params.py
```
