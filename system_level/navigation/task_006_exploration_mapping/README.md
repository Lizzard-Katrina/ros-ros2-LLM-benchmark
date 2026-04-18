# Task 006: Exploration Mapping Translation (ROS 1 to ROS 2)

## 1. Task Overview
This task evaluates the ability to translate a robot's mapping and perception configuration from **ROS 1 (Melodic/Noetic)** to **ROS 2 (Humble/Jazzy)**. 

The source provides a Stretch robot's mapping pipeline using `gmapping` and a standard 2D costmap. The goal is to produce a functionally equivalent ROS 2 configuration using **Nav2** or **SLAM Toolbox** semantics while adhering to modern ROS 2 XML and YAML syntax.
---
source code file:
```https://github.com/hello-robot/stretch_ros/blob/noetic/stretch_navigation/launch/mapping.launch```
```https://github.com/hello-robot/stretch_ros/blob/noetic/stretch_navigation/resources/config/2d/costmap_common_params.yaml```

## 2. Source Context
* **Original SLAM**: Uses `gmapping` to provide the `map -> odom` transform.
* **Original Perception**: Uses a `LaserScan` source named `laser` subscribing to the `/scan` topic.
* **Original Visualization**: Uses ROS 1 `rviz` with a specific `.rviz` config file.

## 3. Translation Requirements (The Holes)

### Hole 1: SLAM Node Migration (`mapping.launch`)
Translate the legacy `<node pkg="gmapping" ... />` tag into a ROS 2 equivalent.
* **Syntax Change**: Replace `type="..."` with `exec="..."`.
* **Semantic Change**: Since `gmapping` is deprecated in ROS 2, use a valid ROS 2 mapping solution such as `slam_toolbox` or a `nav2_map_server` stack.
* **Dependency Change**: Ensure the package name reflects a ROS 2 suite (e.g., `nav2_map_server` or `slam_toolbox`).

### Hole 2: Sensor Stream Migration (`costmap_common_params.yaml`)
Translate the `observation_sources` definition into a ROS 2 Nav2-compatible structure.
* **Topic Consistency**: Maintain the connection to the `/scan` topic.
* **Persistence Logic**: Retain `marking: true` and `clearing: true` to ensure the costmap updates dynamically as the robot moves.
* **Structure**: Ensure the YAML indentation follows the ROS 2 Nav2 plugin parameters standard.

## 4. Verification Logic (Oracle Test)
The provided Oracle test (`test_oracle_translation_006.py`) performs the following checks:
1. **Zero Residue**: Fails if ROS 1 strings like `pkg="gmapping"` or `type="rviz"` are found.
2. **ROS 2 Compliance**: Checks for the presence of the `exec=` attribute in XML.
3. **Semantic Integrity**: Verifies that the `footprint` coordinates and `laser` topic mapping survived the translation process.
4. **Peripheral Update**: Ensures `rviz` was correctly updated to `rviz2`.

---

## How to Run the Benchmark
1. Apply your translation to the `stretch_navigation` package.
2. Execute the Oracle test:
   ```bash
   python3 -m pytest src/test/test_oracle_translation_006.py
