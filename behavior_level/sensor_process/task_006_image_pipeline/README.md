# Task 009: Depth to RGB Point Cloud Radial Conversion (ROS 2)

## 1. Brief Description
This task involves migrating a sophisticated perception node from ROS 1 to ROS 2 Humble. The node, `PointCloudXyzrgbRadialNode`, is responsible for fusing a **Depth Image** and an **RGB Image** into a colored **3D Point Cloud (`sensor_msgs/msg/PointCloud2`)**. 

Key technical challenges include:
* **Multi-Topic Synchronization**: Handling synchronized callbacks for Depth, RGB, and CameraInfo using `message_filters`.
* **Dynamic Re-calibration**: Real-time scaling of Camera Intrinsic ($K$) and Projection ($P$) matrices when Depth and RGB resolutions do not match.
* **Efficient Memory Management**: Utilizing ROS 2 `std::unique_ptr` and Move Semantics (`std::move`) for zero-copy message publishing.
* **Binary Data Manipulation**: Using `PointCloud2Modifier` and `PointCloud2Iterator` to manually construct the point cloud's memory layout.

---
source code
```https://github.com/ros-perception/image_pipeline/blob/rolling/depth_image_proc/src/point_cloud_xyzrgb_radial.cpp```

## 2. Hollowing Strategy
The hollowing focuses on the **`imageCb`** (Image Callback) function, which serves as the central pipeline. To challenge the developer, we hollow out the entire logic starting from input validation to the final publication:

* **The Scaling Logic**: Developers must manually implement the ratio calculation and partial scaling of the CameraInfo matrix.
* **The Conversion Pipeline**: This requires correctly identifying image encodings (16UC1 vs 32FC1) and dispatching them to the correct template-based conversion functions.
* **The ROS 2 Boilerplate**: Implementing the `try-catch` blocks for `cv_bridge` and using the correct ROS 2 publisher API.

**Constraint-Driven Hollowing**: Specific [Style & Logic Constraints] are added to the TODO to prevent "Overfitting" (e.g., preventing the use of loops for matrix scaling to avoid logic errors with the constant $k[8]$).

---

## 3. Oracle Test Design & Expected Outcomes

The Oracle test suite validates both the **ROS 2 framework compliance** and the **mathematical accuracy** of the perception logic.

| Test Case | Design Intent | Expected Outcome |
| :--- | :--- | :--- |
| **`test_camera_info_scaling`** | Ensures only specific focal/principal points are scaled ($k[0, 2, 4, 5]$). | Matches `k[0] *= ratio` and ensures `k[8]` (constant 1.0) is NOT modified. |
| **`test_cv_bridge_safety`** | Enforces robustness against runtime encoding errors. | Presence of `try { ... } catch (cv_bridge::Exception &e)` block. |
| **`test_pcd_modifier`** | Validates the correct setup of the PointCloud2 binary buffer. | Use of `PointCloud2Modifier` with fields `"xyz"` and `"rgb"`. |
| **`test_template_dispatch`** | Checks for support of both 16-bit and 32-bit depth sensors. | Explicit calls to `convertDepthRadial<uint16_t>` and `convertDepthRadial<float>`. |
| **`test_ros2_move_publish`** | Ensures efficient memory handling via C++11 move semantics. | Matches the pattern `publish(std::move(cloud_msg))`. |
| **`test_temporal_accuracy`** | Prevents "Timestamp Drift" in sensor fusion. | The output cloud header must match `depth_msg->header`. |
| **`test_no_legacy_api`** | Scans for deprecated ROS 1 or Boost symbols. | Zero occurrences of `ros::`, `boost::bind`, or `.toSec()`. |
