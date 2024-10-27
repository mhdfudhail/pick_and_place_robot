cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(linkattacher_msgs REQUIRED)

# Function to add an executable with common settings
function(add_common_executable target_name source_file)
  add_executable(${target_name} src/${source_file})
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${target_name} PUBLIC c_std_99 cxx_std_17)
  
  ament_target_dependencies(${target_name}
    moveit_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    rclcpp
    robot_interface
    example_interfaces
    linkattacher_msgs
  )
endfunction()

# ament_target_dependencies(gripper_control ${dependencies})

# Add executables
add_common_executable(test_pose test_pose.cpp)
add_common_executable(multi_pose multi_pose.cpp)
add_common_executable(gripper_control gripper_control.cpp)
add_common_executable(poses_and_service pose_srv.cpp)
add_common_executable(pick_red pick_red.cpp)
add_common_executable(pick_blue pick_blue.cpp)
add_common_executable(pick_green pick_green.cpp)
add_common_executable(pick_all pick_all.cpp)
# add_executable(gripper_control src/gripper_control.cpp)
# ament_target_dependencies(gripper_control ${dependencies})

# Install targets
install(TARGETS 
  test_pose
  multi_pose
  gripper_control
  poses_and_service
  pick_red
  pick_blue
  pick_green
  pick_all
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
