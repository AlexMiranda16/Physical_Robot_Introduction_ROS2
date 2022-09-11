#ifndef SRC_PROXIMITY_DETECTOR_H
#define SRC_PROXIMITY_DETECTOR_H

//#include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
//#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/Image.h>
#include <sensor_msgs/msg/image.hpp>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <numeric>
#include <math.h>


/**
*  The following functions convert the pixel distance value to a metric distance value for the respective direction
*  Return value is in milimeters
*/
double convert_NW(double distance);
double convert_NNW(double distance);
double convert_N(double distance);
double convert_NNE(double distance);
double convert_NE(double distance);
double convert_SW(double distance);
double convert_SSW(double distance);
double convert_S(double distance);
double convert_SSE(double distance);
double convert_SE(double distance);

#endif