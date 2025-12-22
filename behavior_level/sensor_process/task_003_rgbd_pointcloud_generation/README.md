# Task 003 — RGB-D to Point Cloud (Behavior Level / Sensor Process)

## 🔍 Task Description
This task converts synchronized RGB and depth images into a 3D point cloud.
It mimics the `rgbd_launch` behavior from ROS image_pipeline.

### Input Topics:
- `/camera/rgb/image_color`
- `/camera/depth`

### Output Topic:
- `/point_cloud`

---

## 📁 Directory Structure

task_003_rgbd_pointcloud_generation/
│── metadata.json  
│── Dockerfile  
│── setup.sh  
│── ros1_code/
│   ├── rgbd_processor.py      ← contains **one TODO**
│   ├── rgb_subscriber.py      ← contains **one TODO**
│   ├── depth_subscriber.py    ← contains **one TODO**
│   └── launch/
│       └── rgbd_to_pointcloud.launch
│── tests/

---

## 🧩 TODO Locations (for LLM translation tasks)

### 1. rgb_subscriber.py
Goal: create ROS1 subscriber for `/camera/rgb/image_color`

### 2. depth_subscriber.py
Goal: create ROS1 subscriber for `/camera/depth`

### 3. rgbd_processor.py
Goal: using synchronized RGB + depth → generate and publish point cloud

Each file includes:
```
TODO:...
END:...
```

---

## 🐳 Docker Instructions

### Build image
docker build -t task003_rgbd

### Run container
docker run -it --net=host task003_rgbd

### Inside container: run ROS1 version

roslaunch rgbd_task rgbd_to_pointcloud.launch
---

## 🧪 Validation Notes
- No ROS2 code is provided.
- Benchmark focuses on prompting LLM to produce ROS2 version by translating ROS1 logic.

