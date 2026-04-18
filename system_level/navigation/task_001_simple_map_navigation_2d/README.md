# Benchmark Task: ROS 1 to ROS 2 Navigation Stack Migration

## 1. Task Description
This task evaluates an LLM's ability to perform a **System-Level Migration** of a ROS 1 navigation configuration package (`hurba_navigation`) to ROS 2 (Humble). 

Unlike simple code translation, this task requires the model to:
* **Build System Transition:** Identify and transition from `catkin` to `ament_cmake`.
* **System-Level Synchronization:** Ensure dependencies are consistent across `package.xml` and `CMakeLists.txt`.
* **Physical Asset Audit:** Audit the project's directory structure (6 resource folders) and reflect them in the installation logic.

---
source file:
```https://github.com/hungarianrobot/Project-3-Navigation/blob/master/hurba_navigation```

## 2. Design Strategy: Hollowing & TODOs

The benchmark uses a "Deep Hollowing" strategy to force the LLM to reconstruct the package's architecture rather than just fixing syntax.

### **A. package.xml (The Dependency Manifest)**
* **Hole:** All content between `<license>` and `</package>` is removed.
* **TODO:** ``
* **Design Intent:** Tests if the model recognizes that a ROS 2 Nav stack requires `ament_cmake` and specific Nav2 runtime dependencies.

### **B. CMakeLists.txt (The Build Logic)**
* **Hole:** Everything from `find_package(catkin REQUIRED)` to the end of the file is deleted. Legacy ROS 1 comments are stripped.
* **TODO:** `# TODO: Transition to ament_cmake. Find dependencies from package.xml and install all 6 asset directories (config, launch, maps, meshes, rviz, worlds) to share/${PROJECT_NAME}. Conclude with ament_package().`
* **Design Intent:** This is a "Physical Audit" test. The model must look at the actual folders in the workspace and write correct `install` rules while staying in sync with `package.xml`.

---

## 3. Oracle Test Design & Expected Outcomes

The Oracle uses 7 independent Python/Regex tests to validate **Static Architectural Consistency**.

| Test Case | Design Logic | Expected Outcome (Pass Criteria) |
| :--- | :--- | :--- |
| **Buildtool Transition** | Checks for `ament_cmake` buildtool tag. | Correct ROS 2 build tool is declared; `catkin` is removed. |
| **Nav2 Linkage** | Searches for `nav2_bringup` in `package.xml`. | The model recognizes the Nav2-based architecture. |
| **Ament Initialization** | Matches `find_package(ament_cmake REQUIRED)`. | The build script initializes the ROS 2 environment correctly. |
| **Asset Coverage** | Loops through all 6 asset directories. | `config`, `launch`, `maps`, `meshes`, `rviz`, `worlds` are all installed. |
| **Path Mapping** | Verifies `DESTINATION share/${PROJECT_NAME}`. | Assets are mapped to the standard ROS 2 installation space. |
| **Package Registration** | Matches `ament_package()` call. | The package is properly registered in the Ament index. |
| **Cross-File Sync** | Verifies `package.xml` deps exist in CMake. | **Perfect alignment:** No missing `find_package` for declared dependencies. |

---

## 4. How to Run
Run the following command to trigger the static oracle validation:
```bash
docker build -t ros2-nav-migration-test .
