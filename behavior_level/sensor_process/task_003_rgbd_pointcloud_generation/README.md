# Task 003: RGBD PointCloud Generation - Multi-Sensor Fusion

## 1. Brief Description
Implement the core C++ fusion logic for an RGBD camera pipeline. This node merges a Depth map with an RGB image to produce a colored `sensor_msgs/PointCloud2`.
---
source code
```https://github.com/ros-perception/image_pipeline/blob/rolling/depth_image_proc/src/point_cloud_xyzrgb.cpp```

## 2. Excavation Strategy
We have hollowed out the **Image Callback**. The goal is to move beyond simple coding and test **Physical Integrity**:
- **Geometry Awareness**: Scaling camera intrinsics when resizing images.
- **Library Integration**: Utilizing template kernels (`convertDepth<T>`) for performance.
- **Synchronization**: Ensuring the output cloud is temporally and spatially aligned with the depth sensor.

## 3. Oracle Design & Expected Outcomes
| Test | Intent | Passing Outcome |
| :--- | :--- | :--- |
| **Intrinsic Scaling** | Geometric Accuracy | Manual multiplication of focal lengths by resize ratio. |
| **Offset Usage** | Code Efficiency | Explicit `red_offset`, `blue_offset` identification. |
| **Kernel Call** | Library Standard | Concurrent use of `convertDepth` and `convertRgb`. |
| **Header Sync** | Coordinate Safety | `cloud_msg->header = depth_msg->header`. |
| **Memory** | Modern C++ | Usage of `std::make_unique` and `std::move`. |

## 4. Engineering Impact
This task ensures the 3D data produced is metrically accurate. Failing to scale intrinsics or synchronize headers results in "ghosting" or distorted point clouds, which would break downstream SLAM and planning algorithms.
