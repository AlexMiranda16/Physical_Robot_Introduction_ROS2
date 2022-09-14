cmake_minimum_required(VERSION 3.5)
project(ydlidar_ros2_driver)

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
find_package(custom_message REQUIRED)
#find_package(message_generation REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(ydlidar_sdk REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#ament_export_dependencies(
#	roscpp rospy sensor_msgs message_runtime
#	)

add_executable(ydlidar_ros2_driver src/ydlidar_ros2_driver.cpp)

ament_target_dependencies(ydlidar_ros2_driver 
  rclcpp
  rclpy
  sensor_msgs
  std_srvs
  custom_message)

#link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

target_link_libraries(ydlidar_ros2_driver
	${YDLIDAR_SDK_LIBRARIES}
	)


target_include_directories(ydlidar_ros2_driver PUBLIC
  ${YDLIDAR_SDK_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS ydlidar_ros2_driver
  DESTINATION lib/${PROJECT_NAME})

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
