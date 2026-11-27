cmake_minimum_required(VERSION 3.8)
project(mapping_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(lidar_merger_node src/lidar_merger_node.cpp)
target_include_directories(lidar_merger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(lidar_merger_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(lidar_merger_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS})

add_executable(cloud_accumulator_node src/cloud_accumulator_node.cpp)
target_compile_features(cloud_accumulator_node PUBLIC c_std_99 cxx_std_17)
target_link_libraries(cloud_accumulator_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs)

install(TARGETS lidar_merger_node cloud_accumulator_node
  DESTINATION lib/${PROJECT_NAME})

# slam_toolbox mapping sessions (start/stop/save)
install(PROGRAMS scripts/mapping_manager_node.py
  DESTINATION lib/${PROJECT_NAME})

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
