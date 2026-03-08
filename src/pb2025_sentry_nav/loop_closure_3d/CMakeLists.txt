cmake_minimum_required(VERSION 3.8)
project(loop_closure_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(small_gicp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
)

# Loop closure node library
add_library(loop_closure_node SHARED
  src/loop_closure_node.cpp
)
ament_target_dependencies(loop_closure_node ${dependencies})
target_link_libraries(loop_closure_node ${PCL_LIBRARIES} small_gicp::small_gicp)

# Register as component
rclcpp_components_register_node(loop_closure_node
  PLUGIN "loop_closure_3d::LoopClosureNode"
  EXECUTABLE loop_closure_node_exe
)

# Install
install(TARGETS loop_closure_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Temporarily disabled due to Python environment issues
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
