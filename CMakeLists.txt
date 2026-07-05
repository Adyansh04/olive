cmake_minimum_required(VERSION 3.8)
project(olive)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# The scan-matching hot path needs an optimized build; -O0 cannot hold 10 Hz.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration filters)
find_package(whycode_vision REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Common dependencies list
set(COMMON_DEPENDENCIES
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  visualization_msgs
  diagnostic_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
)

# Node: fusion core (LiDAR-inertial front-end + keyframe factor graph)
add_executable(fusion_node
  src/nodes/fusion_node_main.cpp
  src/fusion/fusion_node.cpp
  src/fusion/imu_buffer.cpp
  src/fusion/scan_preprocessor.cpp
  src/fusion/feature_extractor.cpp
  src/fusion/scan_matcher.cpp
  src/fusion/keyframe_map.cpp
  src/fusion/pose_graph.cpp
  src/fusion/wheel_odom_buffer.cpp
  src/fusion/marker_gate.cpp
  src/fusion/health_monitor.cpp
  src/fusion/loop_detector.cpp
)
ament_target_dependencies(fusion_node ${COMMON_DEPENDENCIES} whycode_vision)
target_link_libraries(fusion_node ${PCL_LIBRARIES} gtsam gtsam_unstable)

# Node: monocular visual odometry front-end
add_executable(vo_node
  src/nodes/vo_node_main.cpp
  src/vo/visual_odometry_node.cpp
)
ament_target_dependencies(vo_node ${COMMON_DEPENDENCIES})
target_link_libraries(vo_node ${OpenCV_LIBS})

install(TARGETS
  fusion_node
  vo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz configuration
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install test / calibration scripts (ros2 run olive <script>)
install(PROGRAMS
  scripts/bringup_test.sh
  scripts/drive_test.py
  scripts/square_drive.py
  scripts/ate_eval.py
  scripts/analyze_bag.py
  scripts/plot_results.py
  scripts/marker_demo.py
  scripts/calibrate_marker_range.py
  scripts/imu_bias_relay.py
  scripts/wheel_odom_relay.py
  scripts/endurance_test.py
  scripts/lidar_gate_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_covariance_utils test/test_covariance_utils.cpp)
  ament_target_dependencies(test_covariance_utils geometry_msgs)
  target_link_libraries(test_covariance_utils gtsam)

  ament_add_gtest(test_scan_matcher test/test_scan_matcher.cpp src/fusion/scan_matcher.cpp)
  target_link_libraries(test_scan_matcher ${PCL_LIBRARIES})

  ament_add_gtest(test_marker_anchor test/test_marker_anchor.cpp)
  target_link_libraries(test_marker_anchor gtsam)

  ament_add_gtest(test_marker_observation test/test_marker_observation.cpp)
  target_link_libraries(test_marker_observation gtsam)

  ament_add_gtest(test_marker_gate test/test_marker_gate.cpp src/fusion/marker_gate.cpp)
  target_link_libraries(test_marker_gate gtsam)

  ament_add_gtest(test_pose_graph test/test_pose_graph.cpp src/fusion/pose_graph.cpp)
  target_link_libraries(test_pose_graph gtsam gtsam_unstable)

  ament_add_gtest(test_imu_buffer test/test_imu_buffer.cpp src/fusion/imu_buffer.cpp)
  target_include_directories(test_imu_buffer PRIVATE ${EIGEN3_INCLUDE_DIRS})

  ament_add_gtest(test_wheel_odom_buffer
    test/test_wheel_odom_buffer.cpp src/fusion/wheel_odom_buffer.cpp)
  target_link_libraries(test_wheel_odom_buffer gtsam)

  ament_add_gtest(test_scan_preprocessor
    test/test_scan_preprocessor.cpp src/fusion/scan_preprocessor.cpp src/fusion/imu_buffer.cpp)
  ament_target_dependencies(test_scan_preprocessor sensor_msgs)
  target_link_libraries(test_scan_preprocessor ${PCL_LIBRARIES})

  ament_add_gtest(test_keyframe_map test/test_keyframe_map.cpp src/fusion/keyframe_map.cpp)
  target_link_libraries(test_keyframe_map ${PCL_LIBRARIES} gtsam)

  ament_add_gtest(test_health_monitor test/test_health_monitor.cpp src/fusion/health_monitor.cpp)

  ament_add_gtest(test_loop_detector
    test/test_loop_detector.cpp src/fusion/loop_detector.cpp src/fusion/keyframe_map.cpp)
  target_link_libraries(test_loop_detector ${PCL_LIBRARIES} gtsam)
endif()

ament_package()
