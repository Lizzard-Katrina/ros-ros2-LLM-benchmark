# Task 012: Camera + LiDAR → Sensor Fusion Visualization (Autoware)

## Description
Combine camera and LiDAR outputs to create a fused visualization.  
Subscribe to `/camera/image_raw` and `/lidar/points`, fuse the data, and publish `/fused_markers`.  
Core fusion and projection logic is left as TODO for LLM to fill.


## Expected outcome for each TODO

1. **camera_fusion.cpp**:
   - Fuse camera image with latest LiDAR cloud
   - Project LiDAR points onto image plane if enabled
   - Create fused markers for visualization
   - Publish `/fused_markers` topic
   - Node should compile, subscribers/publishers initialized

2. **lidar_fusion.cpp**:
   - Fuse LiDAR cloud with latest camera image
   - Optionally filter points
   - Prepare markers for visualization
   - Publish `/fused_markers` topic
   - Node should compile, subscribers/publishers initialized

## Validation
- Each TODO can be validated independently with mock camera + LiDAR data
- Full validation: use a recorded ROS bag from camera + LiDAR sensor to visualize fused output in RViz
