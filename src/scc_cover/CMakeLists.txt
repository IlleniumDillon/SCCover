cmake_minimum_required(VERSION 3.5)
project(scc_cover)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FIND_PACKAGES(
  ament_cmake 
  ament_cmake_python 
  rclcpp  
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  OpenCV
  Eigen3
  Boost
  Threads
  scc_message
  scc_simulate
)

include_directories(
  ${CMAKE_SOURCE_DIR}/include/nscc_cover
)

file (GLOB NSCC_COVER_PLAN_SOURCE 
  "src/nscc_cover/*.cpp"
)
BUILD_EXEC(
  TARGET nscc_cover_plan
  SOURCES ${NSCC_COVER_PLAN_SOURCE}
  DEPENDENCIES rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs scc_message OpenCV Eigen3 Boost Threads
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
