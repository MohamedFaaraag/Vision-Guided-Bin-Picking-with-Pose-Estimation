cmake_minimum_required(VERSION 3.8)
project(ur5e_bin_picking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# ── MTC Pick-and-Place Node ──
add_executable(pick_and_place_node
  src/pick_and_place_node.cpp
)
ament_target_dependencies(pick_and_place_node
  rclcpp moveit_ros_planning_interface moveit_task_constructor_core
  tf2_ros tf2_geometry_msgs geometry_msgs control_msgs
)

# ── BehaviorTree Pipeline Node ──
add_executable(bt_pipeline
  src/bt_pipeline.cpp
  src/bt_nodes/detect_objects.cpp
  src/bt_nodes/select_best_candidate.cpp
  src/bt_nodes/plan_grasp.cpp
  src/bt_nodes/execute_pick.cpp
  src/bt_nodes/verify_grasp.cpp
  src/bt_nodes/execute_place.cpp
  src/bt_nodes/check_bin_empty.cpp
)
target_include_directories(bt_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_pipeline
  rclcpp moveit_ros_planning_interface moveit_task_constructor_core
  tf2_ros tf2_geometry_msgs geometry_msgs control_msgs behaviortree_cpp
)

install(TARGETS pick_and_place_node bt_pipeline
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python nodes ──
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/object_detector.py
  scripts/pose_transformer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
