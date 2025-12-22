# Task 005 — Sonar Distance Estimation
**Behavior Level → Sensor Processing**

This task converts raw sonar readings into filtered distance estimates.

---

## 📌 Task Summary

### Input:
- `/sonar/raw`  (Float32)

### Output:
- `/distance_filtered`

The processing node smooths or filters the distance measurement.

---

# 📁 Directory Structure

task_005_sonar_distance_estimation/
│── metadata.json  
│── Dockerfile  
│── setup.sh  
│── ros1_code/
│   ├── sonar_subscriber.py       ← contains **one TODO**
│   ├── filter_node.py            ← contains **one TODO**
│   └── launch/
│       └── sonar_filter.launch
│── tests/

---

# 🧩 TODO Description

### 1. sonar_subscriber.py  
Goal: subscribe to `/sonar/raw`

### 2. filter_node.py  
Goal:  
- subscribe to `/sonar/raw`  
- publish `/distance_filtered`  
- implement simple averaging/smoothing (logic left for LLM)

Each file has exactly:

```
TODO:
END:
```

---

# ⭐ Expected ROS2 Outcome (Brief)

A correct ROS2 translation should:

1. Use `rclpy`
2. Subscribe to `/sonar/raw` as `std_msgs.msg.Float32`
3. Apply smoothing/low-pass filtering
4. Publish filtered distance as `/distance_filtered`
5. Use ROS2 QoS appropriate for sensor data
6. Replace ROS1 init/spin with ROS2 equivalents  
   (`rclpy.init()`, `rclpy.spin()`)

---

# 🐳 Docker Usage

### Build:
docker build -t task005_sonar .


### Run:


docker run -it --net=host task005_sonar


### Inside container:


roslaunch sonar_task sonar_filter.launch


---

# 🧪 Validation
No ROS2 implementation provided.  
This task trains LLMs to translate ROS1 → ROS2 sensor-processing nodes.
