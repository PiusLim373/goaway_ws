cmake_minimum_required(VERSION 3.8)
project(goaway_locomotion)

# The deploy code (ported from unitree_rl_mjlab) builds without extra warning flags.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
# Two programs: goaway_locomotion_node is an SDK program and must not link ROS (the SDK's
# CycloneDDS and ROS's can't share a process); height_map_node is a ROS node.
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(goaway_msgs REQUIRED)

# height_map_node: the L1 cloud and odom -> the policy's height scan
add_executable(height_map_node
  src/height_map/height_map_node.cpp
  src/height_map/rolling_height_map.cpp
  src/height_map/self_filter.cpp
  src/height_map/scan_lookup.cpp
  src/height_map/markers.cpp)
target_include_directories(height_map_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(height_map_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(height_map_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${unitree_go_TARGETS}
  ${goaway_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  Eigen3::Eigen)
install(TARGETS height_map_node
  DESTINATION lib/${PROJECT_NAME})

# The controller only builds for the robot (aarch64 onnxruntime); the PC builds the map node only.
if(NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  message(STATUS "goaway_locomotion_node skipped: only aarch64 has an onnxruntime in thirdparty/")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})
  ament_package()
  return()
endif()

list(APPEND CMAKE_PREFIX_PATH /opt/unitree_robotics)   # unitree_sdk2's install prefix
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-aarch64-1.22.0")

add_executable(goaway_locomotion_node
  src/goaway_locomotion_node.cpp
  src/goaway_locomotion.cpp
  src/State_RLBase.cpp)
target_include_directories(goaway_locomotion_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ONNXRUNTIME_DIR}/include)
target_link_libraries(goaway_locomotion_node
  unitree_sdk2
  Boost::program_options
  yaml-cpp
  fmt::fmt
  Eigen3::Eigen
  rt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0)
# RPATH, not RUNPATH: it beats LD_LIBRARY_PATH, so a sourced ROS can't swap in its own
# libddsc.so. $ORIGIN finds the onnxruntime installed next to the binary.
set_target_properties(goaway_locomotion_node PROPERTIES
  INSTALL_RPATH "/opt/unitree_robotics/lib;\$ORIGIN")
target_link_options(goaway_locomotion_node PRIVATE -Wl,--disable-new-dtags)

install(TARGETS goaway_locomotion_node
  DESTINATION lib/${PROJECT_NAME})
install(FILES ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
  DESTINATION lib/${PROJECT_NAME}
  RENAME libonnxruntime.so.1)   # its soname
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
