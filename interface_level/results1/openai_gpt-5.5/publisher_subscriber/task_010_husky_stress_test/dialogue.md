# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):


# LLM Response

[FILENAME: package.xml]
<?xml version="1.0"?>
<package format="3">
  <name>husky_viz</name>
  <version>0.0.0</version>
  <description>Husky visualization tools and examples.</description>

  <maintainer email="todo@todo.todo">todo</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>rviz2</exec_depend>
  <exec_depend>rqt_gui</exec_depend>
  <exec_depend>rqt_gui_py</exec_depend>
  <exec_depend>rqt_robot_monitor</exec_depend>
  <exec_depend>rqt_runtime_monitor</exec_depend>
  <exec_depend>rqt_tf_tree</exec_depend>
  <exec_depend>urdf</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

[FILENAME: CMakeLists.txt]
cmake_minimum_required(VERSION 3.5)
project(husky_viz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()

