# Task 009: Stretch 3D Perception Pipeline (Point-to-Plane Localization)

## 1. Brief Description
This task evaluates the model's capability to implement a high-precision 3D perception pipeline for the Hello Robot Stretch. The focus is on transforming 2D visual detections into metric 3D spatial coordinates using camera intrinsics, depth data, and geometric plane fitting.

---
source code file:
```https://github.com/hello-robot/stretch_ros/blob/noetic/stretch_deep_perception/nodes/detection_2d_to_3d.py```

---

## 2. Implementation Holes & Logic
The benchmark focuses on two critical stages of the 3D localization pipeline:

### A. Metric Back-Projection (`landmarks_2d_to_3d`)
* **Logic**: Projects 2D pixel landmarks into 3D camera coordinates.
* **Challenge**: The model must handle **millimeter-to-meter** unit conversion and implement the pinhole camera inverse model while managing invalid depth fallbacks.

### B. Analytical Ray-Plane Intersection (`pix_to_plane`)
* **Logic**: Refines object localization by finding where a camera ray hits a pre-calculated 3D plane.
* **Challenge**: Requires an analytical linear algebra solution using the plane normal and distance ($d / (\mathbf{n} \cdot \mathbf{v})$).

---

## 3. Oracle Testcase Design (Hardcore Evaluation)
The Oracle focuses on 6 dimensions of mathematical and engineering integrity:

### 1. Intrinsic Matrix Decomposition (`test_intrinsic_decomposition`)
* **Design**: Verifies correct indexing of the ROS `CameraInfo.K` matrix ($f_x, f_y, c_x, c_y$).
* **Expected Outcome**: Rejects implementations that swap indices or fail to extract core parameters.

### 2. Physical Unit Scaling (`test_depth_unit_scaling`)
* **Design**: Specifically scans for the `/ 1000` or `* 0.001` factor.
* **Expected Outcome**: **Fails** if the model treats raw depth (mm) as metric distance (m).

### 3. Pinhole Identity Check (`test_pinhole_projection_logic`)
* **Design**: Validates the inverse projection formula: $(x - c_x) \times z / f_x$.
* **Expected Outcome**: Ensures 3D point generation follows standard geometric optics.

### 4. Analytical Geometric Solver (`test_ray_plane_analytical_solution`)
* **Design**: Checks for dot products (`@`, `matmul`) and distance-based ray scaling.
* **Expected Outcome**: Validates the implementation of the ray-plane intersection formula.

### 5. Vector Engineering Details (`test_vector_flattening`)
* **Design**: Checks for `.flatten()` to ensure data structure consistency.
* **Expected Outcome**: Prevents dimension mismatch in downstream ROS trajectory planners.

### 6. Robust Noise Reduction (`test_median_noise_reduction`)
* **Design**: Scans for `np.median` implementation in bounding box depth estimation.
* **Expected Outcome**: Ensures the model uses robust statistics instead of simple averages to handle sensor noise/outliers.
