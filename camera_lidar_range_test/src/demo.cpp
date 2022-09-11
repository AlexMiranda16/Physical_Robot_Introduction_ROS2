#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>


void camera_sub(sensor_msgs::msg::LaserScan::SharedPtr camera_msg){

  int n_lasers = camera_msg->ranges.size(); //Number of beams

  float front_laser = camera_msg->ranges[n_lasers/2]; //Distance measured from the frontal laser
  float back_laser = camera_msg->ranges[0]; //Distance measured from the rear laser

  //Prints the distances read from the virtual lidar
  std::cerr << "Front distance: " << front_laser << std::endl;
  std::cerr << "Back distance: " << back_laser << std::endl;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("keyboard_robot_control");

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
  rclcpp::SensorDataQoS qos;
  qos.keep_last(100);
  sub = node->create_subscription<sensor_msgs::msg::LaserScan>("/static_laser", qos, camera_sub);

  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
