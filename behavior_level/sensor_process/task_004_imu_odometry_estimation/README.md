# Task 004 — IMU to Odometry Estimation
**Behavior Level → Sensor Processing**

This task takes IMU raw measurements and converts them into basic odometry using an EKF-like estimator.

---

## 📌 Task Summary

### Input Topic:
- `/imu/data_raw`

### Output Topic:
- `/odom/imu`

This mimics the behavior of robot_localization (EKF integration of IMU).

---

# 📁 Directory Structure

task_004_imu_odometry_estimation/
│── metadata.json  
│── Dockerfile  
│── setup.sh  
│── ros1_code/
│   ├── imu_subscriber.py       ← contains **one TODO**
│   ├── ekf_processor.py        ← contains **one TODO**
│   └── launch/
│       └── imu_to_odom.launch
│── tests/

---

# 🧩 TODO Description

### 1. imu_subscriber.py  
Goal: subscribe to `/imu/data_raw`

### 2. ekf_processor.py  
Goal: initialize EKF-like fusion and publish `/odom/imu`


---

# ⭐ Expected ROS2 Outcome (Brief Summary)

A correct ROS2 translation should:

1. Use `rclpy` and `sensor_msgs.msg.Imu`
2. Implement a node that:
   - subscribes to `/imu/data_raw`
   - processes IMU acceleration & angular velocity
   - computes integrated velocity/pose
   - publishes `nav_msgs.msg.Odometry` to `/odom/imu`
3. Replace rospy callbacks with ROS2 subscription callbacks
4. Replace ROS1 publishers with ROS2 publishers
5. Use parameters & QoS according to ROS2 conventions

---

# 🐳 Docker Build Instructions

### Build
docker build -t task004_imu .

### Run

docker run -it --net=host task004_imu

### Inside container

roslaunch imu_task imu_to_odom.launch
---

# 🧪 Validation
- No ROS2 implementation included.
- Task is designed for benchmarking LLM ability to translate ROS1 → ROS2 logic.


