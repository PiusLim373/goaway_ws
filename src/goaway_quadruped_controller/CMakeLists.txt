cmake_minimum_required(VERSION 3.8)
project(goaway_quadruped_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(htxgrrt_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

# include/ also holds a vendored nlohmann/json (header-only, copied from tmms)
add_executable(goaway_quadruped_controller_node
  src/goaway_quadruped_controller_node.cpp
  src/goaway_quadruped_controller.cpp
  src/api_client.cpp)
target_include_directories(goaway_quadruped_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(goaway_quadruped_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(goaway_quadruped_controller_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${unitree_go_TARGETS}
  ${unitree_api_TARGETS}
  ${htxgrrt_msgs_TARGETS})

install(TARGETS goaway_quadruped_controller_node
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
