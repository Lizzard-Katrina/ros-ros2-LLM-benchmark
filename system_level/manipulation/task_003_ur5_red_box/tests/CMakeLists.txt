cmake_minimum_required(VERSION 3.8)
project(motion_planning)

# 找到依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)

# -------------------------------
# 测试1: test_ik_decision_loop
# -------------------------------
ament_add_gtest(test_ik_decision_loop
  test/test_ik_decision_loop.cpp
)

if(TARGET test_ik_decision_loop)
  # 添加头文件路径
  target_include_directories(test_ik_decision_loop
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/include/motion_planning
  )

  # 添加依赖
  ament_target_dependencies(test_ik_decision_loop
    rclcpp
  )
endif()

# -------------------------------
# 结束
# -------------------------------
ament_package()
