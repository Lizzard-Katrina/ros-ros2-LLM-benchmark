# Task 008 — Camera → Object Detection (YOLO)
Behavior Level → Sensor Processing

This task performs object detection from a camera image stream using a YOLO-based processing node.

---

# 📥 Input
- `/camera/image_raw` (sensor_msgs/Image)

# 📤 Output
- `/detections` (custom or standard detection message)

---

# 📁 Directory Structure
task_008_camera_object_detection_yolo/
│── metadata.json  
│── README.md  
│── Dockerfile  
│── setup.sh  
│── ros1_code/
│   ├── camera_subscriber.py       ← contains one TODO
│   ├── yolo_detector_node.py      ← contains one TODO
│   └── launch/
│       └── yolo_detection.launch
│── tests/

---

# 🧩 TODO Description

### 1. camera_subscriber.py  
Goal:  
Subscribe to `/camera/image_raw` and store/forward frames.

### 2. yolo_detector_node.py  
Goal:  
Process incoming camera frames and publish detection results.


---

# ⭐ Expected ROS2 Outcome (Brief)

A correct ROS2 solution should:

1. Use `rclpy` and `sensor_msgs.msg.Image`.
2. Subscribe to `/camera/image_raw` and convert using `cv_bridge`.
3. Run YOLO inference (OpenCV DNN or YOLOv5/YOLOX, etc.).
4. Publish detection results on `/detections`.
5. Use QoS profiles appropriate for sensor data.

---

# 🐳 Docker Usage

Build:
```docker build -t task008_yolo .
```

Run:

```
docker run -it --net=host task008_yolo
```

Inside container:

```
roslaunch task_008_camera_object_detection_yolo yolo_detection.launch
```
