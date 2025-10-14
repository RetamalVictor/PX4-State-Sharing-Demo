cmake_minimum_required(VERSION 3.5)
project(state_sharing_demo)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###############################################################################
# Dependencies (only what we really use)
###############################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(px4_msgs     REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

###############################################################################
# Library: implementation (src/) + public headers (include/)
###############################################################################
add_library(offboard_control SHARED
  src/offboard_control_srv.cpp
  src/velocity_controller.cpp)

target_include_directories(offboard_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(offboard_control 
  rclcpp 
  px4_msgs 
  tf2 
  tf2_geometry_msgs)
  
# link Eigen with the plain signature to avoid mixing keyword/plain
target_link_libraries(offboard_control
   Eigen3::Eigen
)

###############################################################################
# Tiny launcher that just spins the node
###############################################################################
add_executable(offboard_control_main
  apps/offboard_control_main.cpp)

target_link_libraries(offboard_control_main
  offboard_control)

ament_target_dependencies(offboard_control_main rclcpp)

###############################################################################
# Install
###############################################################################
install(TARGETS
    offboard_control
    offboard_control_main
  LIBRARY DESTINATION lib           # .so
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

###############################################################################
# Export for downstream packages
###############################################################################
ament_export_include_directories(include)
ament_export_libraries(offboard_control)
ament_export_dependencies(rclcpp px4_msgs)

ament_package()
