cmake_minimum_required(VERSION 3.8)
project(custom_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(std_srvs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)

add_executable(plan_node src/plan_node.cpp)
add_executable(test_node src/test_node.cpp)
add_executable(octomap_saver src/octomap_saver.cpp)
target_include_directories(plan_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(octomap_saver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(plan_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(octomap_saver PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  plan_node
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_task_constructor_core"
)
ament_target_dependencies(
  test_node
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_task_constructor_core"
)
ament_target_dependencies(
  octomap_saver
  "moveit_ros_planning_interface"
  "moveit_ros_planning"
  "rclcpp"
  "std_srvs"
  "octomap_msgs"
  "octomap"
)



install(TARGETS plan_node test_node octomap_saver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
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
