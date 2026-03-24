# Task 011: YOLO Object Detector Inference Bridge (ROS 2)

## 1. Brief Description
Migrate the core inference bridge of the Darknet ROS package from ROS 1 to ROS 2 Humble. This component acts as a high-frequency link between raw camera streams and the YOLO deep learning engine. It requires handling image transport, OpenCV integration, and multi-threaded synchronization.
---
source code:
```https://github.com/leggedrobotics/darknet_ros/blob/master/darknet_ros/src/YoloObjectDetector.cpp```

## 2. Hollowing Strategy
We hollow out the two critical ends of the pipeline in `YoloObjectDetector.cpp`:
* **`cameraCallback` (The Sensor Input)**: Handles `cv_bridge` conversion and thread-safe buffering of incoming images.
* **`publishInThread` (The Result Output)**: Serializes Darknet inference results into ROS 2 `BoundingBoxes` messages with proper coordinate scaling.

### Mandatory Constraints (TODO):
* **Thread Safety**: Use `std::lock_guard` (C++11) instead of legacy Boost mutexes.
* **Time Policy**: Explicitly sync output timestamps with the *input image header*, not the current node time.
* **Performance**: Use `std::move` for all message publishing to minimize overhead in high-FPS scenarios.

## 3. Oracle Test Design

| Test Case | Criticality | Logic |
| :--- | :--- | :--- |
| **`test_flexible_mutex_locking`** | High | Detects and bans legacy `boost::` mutexes, ensuring C++11 standard compliance. |
| **`test_timestamp_synchronization`** | **CRITICAL** | Intercepts the common mistake of using `this->now()`. Ensures data temporal consistency. |
| **`test_concurrency_protection`** | High | Uses regex to verify that shared buffers (`roiBoxes_`) are locked *before* access. |
| **`test_move_semantics`** | Medium | Ensures zero-copy optimization is utilized during publishing. |
| **`test_coordinate_scaling`** | Medium | Verifies the math for mapping normalized [0,1] coordinates to pixel [W,H] coordinates. |

## 4. Expected Outcome
The implementation must pass all concurrency checks and maintain strict timestamp alignment. A successful migration will show 0 usage of `ros::Time` and 100% adherence to `rclcpp` logging and clock APIs.
