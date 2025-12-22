# Task 009: LiDAR + IMU → Pose Estimation (LIO-SAM)

## Directory Structure
- `lidar_processor.cpp`: TODO: LiDAR feature extraction & map update
- `imu_processor.cpp`: TODO: IMU integration with LiDAR
- `pose_estimator.h`: TODO: internal data structures initialization
- `main_node.cpp`: Launch PoseEstimator node

## Expected Outcome
- `/velodyne_points` → LiDAR processing → `/map`
- `/imu/data` → IMU fusion → `/odom`
- Each TODO contains one LLM-fillable logic block
- `// END` marks end of fillable section

## Build and Run
```bash
mkdir -p task_009_lio_sam/{ros1_code,tests,docker}
chmod +x task_009_lio_sam/docker/setup.sh

docker build -t task_009_image task_009_lio_sam/docker
docker run -it --rm task_009_image bash
## Expected outcome for each TODO
1. **Constructor TODO (constructor_init.cpp)**: Initialize buffers, sliding window, pose states, optimizer placeholders.
2. **LiDAR processing TODO (lidar_processing.cpp)**: Extract edge/plane features, update local map, compute LiDAR-only odometry increment.
3. **IMU fusion TODO (imu_fusion.cpp)**: Fuse IMU data with LiDAR, update velocity, position, orientation, and odometry.

## Validation
- Each TODO can be validated independently:
    - Constructor: node compiles, subscribers/publishers initialized.
    - LiDAR processing: publish mock point cloud, verify map topic.
    - IMU fusion: feed IMU msg, check odom topic updates.
- Full validation: use ROS bag with `/velodyne_points` and `/imu/data` to compare `/odom` and `/map` outputs.
