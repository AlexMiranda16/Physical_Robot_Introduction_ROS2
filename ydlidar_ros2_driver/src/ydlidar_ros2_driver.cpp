//
// The MIT License (MIT)
//
// Copyright (c) 2020 EAIBOT. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

//#include <ros/ros.h>
//#include "sensor_msgs/LaserScan.h"
//#include "sensor_msgs/PointCloud.h"
//#include "ydlidar_ros_driver/LaserFan.h"
//#include "std_srvs/Empty.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_srvs/srv/empty.hpp"
#include "custom_message/msg/laser_fan.hpp"
#include "src/CYdLidar.h"
#include "ydlidar_config.h"
#include <limits>       // std::numeric_limits
#include <iostream>

#define SDKROSVerision "1.0.2"

CYdLidar laser;

//bool stop_scan(std_srvs::Empty::Request &req,
//               std_srvs::Empty::Response &res) {
  bool stop_scan(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res){
//  ROS_DEBUG("Stop scan");
  auto logger = rclcpp::get_logger("ydlidar_ros2_driver");
  RCLCPP_DEBUG(logger, "Stop scan");
  return laser.turnOff();
}

//bool start_scan(std_srvs::Empty::Request &req,
//                std_srvs::Empty::Response &res) {
  // OLD VERSION bool start_scan(std_srvs::srv::Empty::Request &req, std_srvs::srv::Empty::Response &res){
  bool start_scan(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res){
//  ROS_DEBUG("Start scan");
  auto logger = rclcpp::get_logger("ydlidar_ros2_driver");
  RCLCPP_DEBUG(logger, "Start scan");
  return laser.turnOn();
}



int main(int argc, char **argv) {
//  ros::init(argc, argv, "ydlidar_ros_driver");
//  ROS_INFO("YDLIDAR ROS Driver Version: %s", SDKROSVerision);
//  ros::NodeHandle nh;
//  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
//  ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud>("point_cloud",
//                          1);
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("ydlidar_ros2_driver");
  RCLCPP_INFO(nh->get_logger(), "YDLIDAR ROS Driver Version: %s", SDKROSVerision);
  
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub = nh->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pc_pub = nh->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", 1);



  //ydlidar::os_init();


/*
//  ros::Publisher laser_fan_pub =
//    nh.advertise<ydlidar_ros_driver::LaserFan>("laser_fan", 1);
*/

//  ros::NodeHandle nh_private("~");
  std::string str_optvalue = "/dev/ttyYDLidar";
//  nh_private.param<std::string>("port", str_optvalue, "/dev/ydlidar");
  //nh->declare_parameter("port", "/dev/ydlidar");
  nh->get_parameter("port", str_optvalue);

  ///lidar port
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(),
                    str_optvalue.size());

  ///ignore array
//  nh_private.param<std::string>("ignore_array", str_optvalue, "");
  //nh->declare_parameter("ignore_array", "");
  nh->get_parameter("ignore_array", str_optvalue);
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(),
                    str_optvalue.size());

  std::string frame_id = "laser_frame";
//  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
  //nh->declare_parameter("frame_id", "laser_frame");
  nh->get_parameter("frame_id", frame_id);

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 128000;
//  nh_private.param<int>("baudrate", optval, 230400);
  //nh->declare_parameter("baudrate", 230400);
  nh->get_parameter("baudrate", optval);
  //printf("%d\n", optval);
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TRIANGLE;
//  nh_private.param<int>("lidar_type", optval, TYPE_TRIANGLE);
  //nh->declare_parameter("lidar_type", TYPE_TRIANGLE);
  nh->get_parameter("lidar_type", optval);
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
//  nh_private.param<int>("device_type", optval, YDLIDAR_TYPE_SERIAL);
  //nh->declare_parameter("device_type", YDLIDAR_TYPE_SERIAL);
  nh->get_parameter("device_type", optval);
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 9;
//  nh_private.param<int>("sample_rate", optval, 9);
  //nh->declare_parameter("sample_rate", 9);
  nh->get_parameter("sample_rate", optval);
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count RCLCPP_ERROR(nh->get_logger(), "%s\n", laser.DescribeError());
  optval = 4;
//  nh_private.param<int>("abnormal_check_count", optval, 4);
  //nh->declare_parameter("abnormal_check_count", 4);
  nh->get_parameter("abnormal_check_count", optval);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));


  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
//  nh_private.param<bool>("resolution_fixed", b_optvalue, true);
  //nh->declare_parameter("resolution_fixed", true);
  nh->get_parameter("resolution_fixed", b_optvalue);
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
//  nh_private.param<bool>("reversion", b_optvalue, true);
  //nh->declare_parameter("reversion", true);
  nh->get_parameter("reversion", b_optvalue);
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
//  nh_private.param<bool>("inverted", b_optvalue, true); RCLCPP_ERROR(nh->get_logger(), "%s\n", laser.DescribeError());
  //nh->declare_parameter("inverted", true);
  nh->get_parameter("inverted", b_optvalue);
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
//  nh_private.param<bool>("auto_reconnect", b_optvalue, true);
  //nh->declare_parameter("auto_reconnect", true);
  nh->get_parameter("auto_reconnect", b_optvalue);
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = false;
//  nh_private.param<bool>("isSingleChannel", b_optvalue, false);
  //nh->declare_parameter("isSingleChannel", true);
  nh->get_parameter("isSingleChannel", b_optvalue);
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
//  nh_private.param<bool>("intensity", b_optvalue, false);
  //nh->declare_parameter("intensity", true);
  nh->get_parameter("intensity", b_optvalue);
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = false; RCLCPP_ERROR(nh->get_logger(), "%s\n", laser.DescribeError());
//  nh_private.param<bool>("support_motor_dtr", b_optvalue, false);
  //nh->declare_parameter("support_motor_dtr", true);
  nh->get_parameter("support_motor_dtr", b_optvalue);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
//  nh_private.param<float>("angle_max", f_optvalue, 180.f);
  //nh->declare_parameter("angle_max", 180.f);
  nh->get_parameter("angle_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
//  nh_private.param<float>("angle_min", f_optvalue, -180.f);
  //nh->declare_parameter("angle_min", -180.f);
  nh->get_parameter("angle_min", f_optvalue);
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 16.f;
//  nh_private.param<float>("range_max", f_optvalue, 16.f);
  //nh->declare_parameter("range_max", 16.f);
  nh->get_parameter("range_max", f_optvalue); RCLCPP_ERROR(nh->get_logger(), "%s\n", laser.DescribeError());
  f_optvalue = 0.1f;
//  nh_private.param<float>("range_min", f_optvalue, 0.1f);
  //nh->declare_parameter("range_min", 0.1f);
  nh->get_parameter("range_min", f_optvalue);
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 10.f;
//  nh_private.param<float>("frequency", f_optvalue, 10.f);
  //nh->declare_parameter("frequency", 10.f);
  nh->get_parameter("frequency", f_optvalue);
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  bool invalid_range_is_inf = false;
//  nh_private.param<bool>("invalid_range_is_inf", invalid_range_is_inf,
//                         invalid_range_is_inf);
  //nh->declare_parameter("invalid_range_is_inf", invalid_range_is_inf);
  nh->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

  bool point_cloud_preservative = false;
//  nh_private.param<bool>("point_cloud_preservative", point_cloud_preservative,
//                         point_cloud_preservative);
  //nh->declare_parameter("point_cloud_preservative", point_cloud_preservative);
  nh->get_parameter("point_cloud_preservative", point_cloud_preservative);

//  ros::ServiceServer stop_scan_service = nh.advertiseService("stop_scan", stop_scan);
//  ros::ServiceServer start_scan_service = nh.advertiseService("start_scan",
//                                          start_scan);
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_scan_service = nh->create_service<std_srvs::srv::Empty>("stop_scan", &stop_scan);
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_scan_service = nh->create_service<std_srvs::srv::Empty>("start_scan", &start_scan);

  std::cout << "ANTES DO INITIALIZE\n";

  // initialize SDK and LiDAR
  bool ret = laser.initialize();
  
  std::cout << "DEPOIS DO INITIALIZE\n";

  if (ret) {//success
    //Start the device scanning routine which runs on a separate thread and enable motor.
    std::cout << "ANTES DO TURN ON\n";
    ret = laser.turnOn();
    std::cerr << "DEPOIS DO TURN ON\n";
  } else {
//    ROS_ERROR("%s\n", laser.DescribeError());
      RCLCPP_ERROR(nh->get_logger(), "%s\n", laser.DescribeError());
  }

//  ros::Rate r(30);
  rclcpp::Rate r(30);

/*
  while(rclcpp::ok()){
    laser.turnOn();
    rclcpp::spin_some(nh);
  }

  //rclcpp::spin(nh);
*/

  while (ret && rclcpp::ok()) {//ros::ok()) {
    LaserScan scan;
  std::cerr << "DEPOIS DO while\n";

    if (laser.doProcessSimple(scan)) {
      std::cerr << "DEPOIS DO if\n";
//      sensor_msgs::LaserScan scan_msg;
//      sensor_msgs::PointCloud pc_msg;
      //ydlidar_ros_driver::LaserFan fan;
//      ros::Time start_scan_time;
      sensor_msgs::msg::LaserScan scan_msg;
      sensor_msgs::msg::PointCloud pc_msg;
//      start_scan_time.sec = scan.stamp / 1000000000ul;
//      start_scan_time.nsec = scan.stamp % 1000000000ul;
//      scan_msg.header.stamp = start_scan_time;

      scan_msg.header.stamp.sec = scan.stamp / 1000000000ul;
      scan_msg.header.stamp.nanosec = scan.stamp % 1000000000ul;
      scan_msg.header.frame_id = frame_id;
      pc_msg.header = scan_msg.header;
      //fan.header = scan_msg.header;
      scan_msg.angle_min = (scan.config.min_angle);
      scan_msg.angle_max = (scan.config.max_angle);
      scan_msg.angle_increment = (scan.config.angle_increment);
      scan_msg.scan_time = scan.config.scan_time;
      scan_msg.time_increment = scan.config.time_increment;
      scan_msg.range_min = (scan.config.min_range);
      scan_msg.range_max = (scan.config.max_range);
      //fan.angle_min = (scan.config.min_angle);
      //fan.angle_max = (scan.config.max_angle);
      //fan.scan_time = scan.config.scan_time;
      //fan.time_increment = scan.config.time_increment;
      //fan.range_min = (scan.config.min_range);
      //fan.range_max = (scan.config.max_range);
std::cerr << "antes do size\n";
      int size = (scan.config.max_angle - scan.config.min_angle) /
                 scan.config.angle_increment + 1;
      scan_msg.ranges.resize(size,
                             invalid_range_is_inf ? std::numeric_limits<float>::infinity() : 0.0);
      scan_msg.intensities.resize(size);
      pc_msg.channels.resize(2);
      int idx_intensity = 0;
      pc_msg.channels[idx_intensity].name = "intensities";
      int idx_timestamp = 1;
      pc_msg.channels[idx_timestamp].name = "stamps";

std::cerr << "Antes dio for\n";

      for (size_t i = 0; i < scan.points.size(); i++) {
        std::cerr << "DEPOIS DO for\n";
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle) /
                              scan.config.angle_increment);

        if (index >= 0 && index < size) {
          if (scan.points[i].range >= scan.config.min_range) {
            scan_msg.ranges[index] = scan.points[i].range;
            scan_msg.intensities[index] = scan.points[i].intensity;
          }
        }

        if (point_cloud_preservative ||
            (scan.points[i].range >= scan.config.min_range &&
             scan.points[i].range <= scan.config.max_range)) {
//          geometry_msgs::Point32 point;
          geometry_msgs::msg::Point32 point;
          point.x = scan.points[i].range * cos(scan.points[i].angle);
          point.y = scan.points[i].range * sin(scan.points[i].angle);
          point.z = 0.0;
          pc_msg.points.push_back(point);
          pc_msg.channels[idx_intensity].values.push_back(scan.points[i].intensity);
          pc_msg.channels[idx_timestamp].values.push_back(i * scan.config.time_increment);
        }

        //fan.angles.push_back(scan.points[i].angle);
        //fan.ranges.push_back(scan.points[i].range);
        //fan.intensities.push_back(scan.points[i].intensity);
      }

//      scan_pub.publish(scan_msg);
//      pc_pub.publish(pc_msg);
      scan_pub->publish(scan_msg);
      pc_pub->publish(pc_msg);
      //laser_fan_pub.publish(fan);

    } else {
//      ROS_ERROR("Failed to get Lidar Data");
  std::cerr << "SIM, É ESTE ERRO\n";
      RCLCPP_ERROR(rclcpp::get_logger("ydlidar_ros2_driver"), "Failed to get Lidar Data");
    }

//    r.sleep();
//    ros::spinOnce();
    r.sleep();
    rclcpp::spin_some(nh);
  }

  laser.turnOff();
//  ROS_INFO("[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  RCLCPP_INFO(rclcpp::get_logger("ydlidar_ros2_driver"), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.disconnecting();


  return 0;
}