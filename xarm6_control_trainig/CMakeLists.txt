cmake_minimum_required(VERSION 3.8)
project(xarm6_control_trainig)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_core REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_planner REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)



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

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/tennis_ball_position_publisher_joint_position.py
  scripts/tennis_ball_position_publisher_joint_velocity.py
  scripts/tennis_ball_position_publisher_joint_cylinder_velocity.py
  scripts/reset_sim_tennis_ball.py
  scripts/xarm_joint_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
add_library(xarm_planner STATIC IMPORTED)
set_target_properties(xarm_planner PROPERTIES IMPORTED_LOCATION xarm_planner/include/xarm_planner/xarm_planner.h)

add_executable(controller_moveit src/controller_moveit.cpp)
target_include_directories(controller_moveit PUBLIC include)
ament_target_dependencies(controller_moveit
  ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
  xarm_msgs
  xarm_planner)

# Install the executable
install(TARGETS 
    controller_moveit
    DESTINATION lib/${PROJECT_NAME}
)



install(
  DIRECTORY
    urdf
    launch
    scripts
  DESTINATION
    share/${PROJECT_NAME}/
)
ament_package()
