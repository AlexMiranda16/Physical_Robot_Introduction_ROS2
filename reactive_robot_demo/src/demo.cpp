#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <cmath>

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;

int curr_state = 0, last_state = 0; //Aux for the current and last states to ensure that the commands are published only one 
                                    //time and not multiple times of the same command

void camera_sub(sensor_msgs::msg::LaserScan::SharedPtr camera_msg){

  std_msgs::msg::String command_msg;

  int n_lasers = camera_msg->ranges.size(); //Number os beams

  float avoid_distance = 200.00; //Distance to where the robot should rotate to avoid colision with an object
  float front_laser = camera_msg->ranges[n_lasers/2]; //Distance measured from the frontal laser
  float front_left_laser = camera_msg->ranges[n_lasers/2 - round( 20/(360/n_lasers) ) ]; //Distance measured from the frontal laser angled to the left
  float front_right_laser = camera_msg->ranges[n_lasers/2 + round( 20/(360/n_lasers) ) ]; //Distance measured from the frontal laser angled to the right

  //Decide the movement to make
  //Rotate
  if ( (std::isnan(front_laser)) || (front_laser < avoid_distance) || (front_left_laser < avoid_distance) || (front_right_laser < avoid_distance)){

    //Rotate left command
    command_msg.data = "L0050 R0050";
    curr_state = 1;
  }
  //Move forward
  else{

    //Move forward command
    command_msg.data = "L1050 R0050";
    curr_state = 2;
  }

  //Check if there is a new command
  if ( curr_state != last_state){

    //Publishing the command
    pub->publish(command_msg);
    last_state = curr_state;
  }
}


int main(int argc, char ** argv)
{


  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("keyboard_robot_control");

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
  rclcpp::SensorDataQoS qos;
  qos.keep_last(100);
  sub = node->create_subscription<sensor_msgs::msg::LaserScan>("/static_laser", qos, camera_sub);

  pub = node->create_publisher<std_msgs::msg::String>("/motor_speed", 1);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
