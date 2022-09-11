cmake_minimum_required(VERSION 3.5)
project(fisheye_camera_simulated_sensors_ros2)

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
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(GTK REQUIRED gtk+-3.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_message REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

set(raspicam_DIR "/usr/local/lib/cmake")
find_package(raspicam REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(simulated_lidar src/simulated_lidar.cpp)
add_executable(object_detection src/object_detection.cpp)
add_executable(camera_calibration src/calibration_utils.cpp)
add_executable(proximity_sensor src/proximity_sensor.cpp)

ament_target_dependencies(simulated_lidar
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)

ament_target_dependencies(object_detection
  rclcpp
  custom_message
)

ament_target_dependencies(camera_calibration
  rclcpp
)

ament_target_dependencies(proximity_sensor
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)

#Isto vai para o target_include_directories
#include_directories(
#  include
#  ${GTK_INCLUDE_DIRS}
#)

target_link_libraries(simulated_lidar
  ${OpenCV_LIBRARIES}
  ${raspicam_CV_LIBS}
)

target_link_libraries(object_detection
  ${OpenCV_LIBRARIES}
  ${raspicam_CV_LIBS}
)

target_link_libraries(camera_calibration
  ${OpenCV_LIBRARIES}
  ${raspicam_CV_LIBS}
)

target_link_libraries(proximity_sensor
  ${OpenCV_LIBRARIES}
  ${raspicam_CV_LIBS}
)


target_include_directories(camera_calibration PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(object_detection PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(proximity_sensor PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(simulated_lidar PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS 
  camera_calibration
  object_detection
  simulated_lidar
  proximity_sensor
  DESTINATION lib/${PROJECT_NAME})


#ament_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES serp
#  ament_export_dependencies geometry_msgs roscpp message_runtime 
  #  DEPENDS system_lib
#)



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
