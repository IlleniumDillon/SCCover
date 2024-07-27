cmake_minimum_required(VERSION 3.5)
project(scc_simulate)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

############################ RVIZ PLUGIN ############################
FIND_PACKAGES(
  ament_cmake
  ament_cmake_ros
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  pluginlib
  rclcpp
  resource_retriever
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
  rviz_default_plugins
  scc_message
  OpenCV
)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

include_directories(
  ${CMAKE_SOURCE_DIR}/include/rviz_plugin
)

file(GLOB SCC_RVIZ_PLUGIN_SOURCE "src/rviz_plugin/*.cpp")
file(GLOB SCC_RVIZ_PLUGIN_HEADERS "include/rviz_plugin/*.hpp")
file(GLOB SCC_PLUGIN_ICONS "icons/*.png")

BUILD_RVIZ_PLUGIN(
  TARGET scc_rviz_plugin
  SOURCES ${SCC_RVIZ_PLUGIN_SOURCE}
  HEADERS ${SCC_RVIZ_PLUGIN_HEADERS}
  DEPENDENCIES rviz_common rviz_rendering rviz_ogre_vendor geometry_msgs image_transport interactive_markers laser_geometry map_msgs nav_msgs pluginlib rclcpp resource_retriever sensor_msgs tf2 tf2_geometry_msgs tf2_ros urdf visualization_msgs rviz_default_plugins scc_message OpenCV
)

INSTALL_RVIZ_PLUGIN(
  PLUGIN_XML "plugins_description.xml"
  ICONS ${SCC_PLUGIN_ICONS}
)
############################ RVIZ PLUGIN ############################

############################ GAZEBO PLUGIN ############################
# FIND_PACKAGES(
#   ament_cmake 
#   rclcpp  
#   gazebo_dev
#   gazebo_msgs
#   gazebo_ros
#   geometry_msgs
#   nav_msgs
#   sensor_msgs
#   std_msgs
#   std_srvs
#   tf2
#   tf2_geometry_msgs
#   tf2_ros
#   trajectory_msgs
#   scc_message
# )

# include_directories(
#   ${CMAKE_SOURCE_DIR}/include/gazebo_plugin
# )
############################ GAZEBO PLUGIN ############################

INSTALL_DIR(
  launch 
  rviz
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
