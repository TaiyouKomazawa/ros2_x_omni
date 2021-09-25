cmake_minimum_required(VERSION 3.5)
project(ros2_x_omni)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros2_x_omni_msgs REQUIRED)

add_executable(x_omni_node src/x_omni_node.cpp)

ament_target_dependencies(x_omni_node
  rclcpp
  geometry_msgs
  ros2_x_omni_msgs
)

target_link_libraries(x_omni_node ${CMAKE_CURRENT_SOURCE_DIR}/SerialBridge/bin/libSerialBridge.a)

target_include_directories(x_omni_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/SerialBridge/src)

install(TARGETS x_omni_node
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(scripts)
install(PROGRAMS
  scripts/odom_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME x_odom_node
)

install(PROGRAMS
  launch/x_omni.launch.py
  DESTINATION share/${PROJECT_NAME}
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
