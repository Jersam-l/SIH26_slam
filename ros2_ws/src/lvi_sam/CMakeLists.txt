cmake_minimum_required(VERSION 3.8)
project(lvi_sam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED COMPONENTS
  common
  io
  filters
  kdtree
  registration
)

find_package(ament_index_cpp REQUIRED)
find_package(lvi_sam_interfaces REQUIRED)

include_directories(
  BEFORE
  ${CMAKE_SOURCE_DIR}/../../../gtsam-4.0.2/gtsam/3rdparty/Eigen
  /opt/ros/humble/include/pcl_conversions/pcl_conversions
  /opt/ros/humble/include/pcl_msgs
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# --- Common ament dependencies for all targets ---
set(AMENT_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  ament_index_cpp
  lvi_sam_interfaces
)

# --- featureExtraction ---
add_executable(lvi_sam_featureExtraction
  src/lidar_odometry/featureExtraction.cpp
)
ament_target_dependencies(lvi_sam_featureExtraction ${AMENT_DEPS})
target_link_libraries(lvi_sam_featureExtraction
  ${PCL_LIBRARIES}
)

# --- imuPreintegration ---
add_executable(lvi_sam_imuPreintegration
  src/lidar_odometry/imuPreintegration.cpp
)
ament_target_dependencies(lvi_sam_imuPreintegration ${AMENT_DEPS})
target_link_libraries(lvi_sam_imuPreintegration
  ${PCL_LIBRARIES}
  gtsam
  gtsam_unstable
)

# --- imageProjection ---
add_executable(lvi_sam_imageProjection
  src/lidar_odometry/imageProjection.cpp
)
ament_target_dependencies(lvi_sam_imageProjection ${AMENT_DEPS})
target_link_libraries(lvi_sam_imageProjection
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# --- mapOptmization ---
add_executable(lvi_sam_mapOptmization
  src/lidar_odometry/mapOptmization.cpp
)
ament_target_dependencies(lvi_sam_mapOptmization ${AMENT_DEPS})
target_link_libraries(lvi_sam_mapOptmization
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  OpenMP::OpenMP_CXX
  gtsam
)

install(TARGETS
  lvi_sam_featureExtraction
  lvi_sam_imuPreintegration
  lvi_sam_imageProjection
  lvi_sam_mapOptmization
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
