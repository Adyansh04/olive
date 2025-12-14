cmake_minimum_required(VERSION 3.8)
project(olive)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native -DNDEBUG)
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
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration filters)
find_package(GTSAM REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
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
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
)

# Node: Lidar Odometry 
add_executable(lidar_odom_node
  src/nodes/lidar_odom_node.cpp
  src/lidar_odom/lidar_odometry.cpp
  src/lidar_odom/feature_extractor.cpp
  src/lidar_odom/feature_registration.cpp
  src/lidar_odom/local_map.cpp
)
ament_target_dependencies(lidar_odom_node ${COMMON_DEPENDENCIES})
target_link_libraries(lidar_odom_node ${PCL_LIBRARIES})

# Install targets
install(TARGETS
  lidar_odom_node
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

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

#####################################
# Example: If you don't have a target yet, create a dummy one or your main library
add_library(${PROJECT_NAME} SHARED
  src/dummy.cpp # You might need to create an empty dummy.cpp if you have no sources yet
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${COMMON_DEPENDENCIES}
  Eigen3
)
####################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
