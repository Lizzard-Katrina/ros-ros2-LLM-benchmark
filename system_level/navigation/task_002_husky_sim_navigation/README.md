# Task 002: Husky Navigation System Migration (ROS 1 to ROS 2)

## 1. Brief Description
This task targets the migration of a robotic navigation sub-system (Clearpath Husky) from ROS 1 Melodic/Noetic to ROS 2 Humble. The core objective is to move from the legacy `move_base` architecture to the modular **Nav2** stack. This is a **System-Level Migration** test, requiring the model to maintain functional and structural consistency across the package manifest, build system, and launch orchestration.
---
source file:
```https://github.com/husky/husky/blob/noetic-devel/husky_navigation```

## 2. "Hole" Design Strategy (TODO Logic)
The "holes" in this task are designed with **Coarse Granularity** to test the model's ability to reconstruct system architecture rather than just translating line-by-line:

- **package.xml**: The entire dependency and export section is removed. The model must "guess" and include the correct Nav2 suite (e.g., `nav2_bringup`, `nav2_planner`) based on the task context.
- **CMakeLists.txt**: The installation and package registration logic is removed. The model must realize that for a navigation package, physical directories (`config`, `launch`, `maps`) MUST be installed to the `share` space, or the system will fail at runtime.
- **move_base.launch**: The entire XML content is replaced with a single TODO. This forces the model to:
    1.  Decide whether to use ROS 2 XML or Python Launch (Python is expected for high scores).
    2.  Map old `rosparam` loads to new `Node` parameters.
    3.  Re-implement the specific business logic (`no_static_map` argument).

## 3. Oracle Test Case Design & Expected Outcome

| Test Case | Design Rationale (Concept) | Expected Outcome (Pass Condition) |
| :--- | :--- | :--- |
| `test_pkg_format_and_buildtool` | Validates the foundational build system transition. | Must find `<package format="3">` and `<buildtool_depend>ament_cmake</buildtool_depend>`. |
| `test_nav_stack_replacement` | Validates functional mapping from ROS 1 to ROS 2. | Must contain `nav2_bringup` and `slam_toolbox` (or equivalent) while removing `move_base`. |
| `test_cmake_asset_installation` | Validates **System Visibility**. In ROS 2, files aren't "found" unless installed. | Regex must match `install(DIRECTORY ...)` covering `config`, `launch`, AND `maps`. |
| `test_launch_path_resolution` | Validates the move from static to dynamic resource locating. | Must use `get_package_share_directory('husky_navigation')` or `find-pkg-share`. |
| `test_launch_conditional_logic` | Validates **Functional Parity**. Checks if the model actually read the ROS 1 logic. | Must implement a ROS 2 condition (e.g., `IfCondition`) based on the `no_static_map` argument. |
| `test_system_dependency_sync` | Validates **Cross-File Consistency**. This is the ultimate "System" check. | Any package called in the Launch file (e.g., `nav2_bt_navigator`) MUST exist in `package.xml`. |
