cmake_minimum_required(VERSION 3.8)
project(turtlesim_hunter_target_routine)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(custom_turtle_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)


# Define Executables:
# add executable for the first node
add_executable(turtle_controller_exe src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller_exe rclcpp geometry_msgs turtlesim custom_turtle_interfaces)

add_executable(turtle_spawner_exe src/turtle_spawner.cpp)
ament_target_dependencies(turtle_spawner_exe rclcpp turtlesim custom_turtle_interfaces)

install(TARGETS
  turtle_controller_exe
  turtle_spawner_exe
  
  DESTINATION lib/${PROJECT_NAME})


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
