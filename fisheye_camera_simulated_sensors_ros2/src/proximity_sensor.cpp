#include "proximity_sensor.h"

const double pi = 3.14159265358979323846;

//angle at -1/4*pi
double convert_NW(double distance){
    //y = (-118.7824*x)/(-258.2981 + x)
    return (-118.7824 * distance)/(-258.2981 + distance);
}

//angle at -1/8*pi
double convert_NNW(double distance){
    //y = (-106.463*x)/(-261.4368 + x)
    return (-106.463 * distance)/(-261.4368 + distance);
}

//angle at 0
double convert_N(double distance){
    //y = (-108.4156*x)/(-264.6905 + x)
    return (-108.4156 * distance)/(-264.6905 + distance);
}

//angle at 1/8*pi
double convert_NNE(double distance){
	//y = (-122.4684*x)/(-266.7484 + x)
    return (-122.4684 * distance)/(-266.7484 + distance);
}

//angle at 1/4*pi
double convert_NE(double distance){
	//y = (-104.0271*x)/(-256.1551 + x)
    return (-104.0271 * distance)/(-256.1551 + distance);
}

//angle at -3/4*pi
double convert_SW(double distance){
    //y = (-119.897*x)/(-224.2417 + x)
    return (-119.897 * distance)/(-224.2417 + distance);
}

//angle at -7/8*pi
double convert_SSW(double distance){
    //y = (-124.4037*x)/(-220.4973 + x)
    return (-124.4037 * distance)/(-220.4973 + distance);
}

//angle at pi
double convert_S(double distance){
    //y = (-127.9308*x)/(-220.4707 + x)
    return (-127.9308 * distance)/(-220.4707 + distance);
}

//angle at 7/8*pi
double convert_SSE(double distance){
	//y = (-127.1675*x)/(-221.9849 + x)
    return (-127.1675 * distance)/(-221.9849 + distance);
}

//angle at 3/4*pi
double convert_SE(double distance){
	//y = (-118.8645*x)/(-225.261 + x)
    return (-118.8645 * distance)/(-225.261 + distance);
}
	
	
int main(int argc, char** argv)
{
//    ros::init(argc, argv, "proximity_sensor_node");
//    ros::NodeHandle n_public("~");
//    ros::Rate rate(20); //rate set at 20 Hz
    rclcpp::init(argc, argv);
    auto n_public = rclcpp::Node::make_shared("proximity_sensor");
    rclcpp::Rate rate(20); //rate set to 20Hz

    bool debug_mode = false;
//    n_public.getParam("debug", debug_mode);
    n_public->get_parameter("debug", debug_mode);
    //std::cout << "Debug mode is set to " << debug_mode << "\n";

    bool publish_raw_image = false;
//    n_public.getParam("publish_raw_image", publish_raw_image);
    n_public->get_parameter("public_raw_image", publish_raw_image);
    //std::cout << "Image publishing is set to " << publish_raw_image << "\n";

//    ros::Publisher laser_scan_pub = n_public.advertise<sensor_msgs::LaserScan>("/static_laser", 1);
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub;
    laser_scan_pub = n_public->create_publisher<sensor_msgs::msg::LaserScan>("/static_laser", 1);

    image_transport::ImageTransport it(n_public);
    image_transport::Publisher pub_img = it.advertise("/camera/image", 1);
//    sensor_msgs::ImagePtr img_msg;
    sensor_msgs::msg::Image::Ptr img_msg;

    // Create a VideoCapture object and open the input file
    // If the input is the web camera, pass 0 instead of the video file name
    cv::VideoCapture cap(0);

    // Check if camera opened successfully
    if(!cap.isOpened())
    {
//        ROS_WARN_STREAM("Error opening video stream or file");
        RCLCPP_WARN_STREAM(n_public->get_logger(), "Error opening video stream or file");
        return -1;
    }

    while(rclcpp::ok())//ros::ok())
    {
        cv::Mat frame;
        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty()) break;

        if(publish_raw_image){
            //sensor_msgs::Image img_msg;
            //std_msgs::Header header;
            //header.stamp = ros::Time::now();
            //cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
            //img_bridge.toImageMsg(img_msg);

//            img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            pub_img.publish(img_msg);
            cv::waitKey(1);

        }

        // Display the resulting frame
        if(debug_mode){
            cv::imshow("Original Frame", frame);
            cv::waitKey(1);
        }

        cv::Mat img_gray;
        cv::cvtColor(frame, img_gray, cv::COLOR_BGR2GRAY);
        if(debug_mode){
            cv::imshow("Grayscale Frame", img_gray);
            cv::waitKey(1);
        }

        cv::Mat img_bilat;
        cv::bilateralFilter(img_gray, img_bilat, 11, 75, 75);
        if(debug_mode){
            cv::imshow("Bilat Frame", img_bilat);
            cv::waitKey(1);
        }

        cv::Mat img_canny;
        cv::Canny(img_bilat, img_canny, 60, 60, 3);
        if(debug_mode){
            cv::imshow("Canny Frame", img_canny);
            cv::waitKey(1);
        }

        int frame_width = frame.size().width;
        int frame_height = frame.size().height;

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(img_canny, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
        cv::Scalar color = cv::Scalar(0,0,255);
        
        // Vectors are set up in the following order: {NW, NNW, N, NNE, NE, SW, SSW, S, SSE, SE}
        std::vector<double> distances(10, 300);
        std::vector<cv::Point> detected_points(10, cv::Point(0,0));

        for(size_t i = 0; i < contours.size(); i++){
            //drawContours(frame, contours, (int) i, color);
            //std::cout << "Contour " << i << " is " << contours.at(i);
            for(size_t j = 0; j < contours.at(i).size(); j++){
                int x_diff = contours.at(i).at(j).x - frame_width/2;
                int y_diff = contours.at(i).at(j).y - frame_height/2;
                double distance = cv::sqrt(x_diff*x_diff + y_diff*y_diff);
                double angle = acos((x_diff * -1 + y_diff * 0)/(distance * 1));
                
                if(y_diff > 0){angle = -angle;};

                //objects closer than the minimum distance will not be detected so the robot chassis does not cause interference
                if(distance > 135.0){
                    int index = -1;
                    if(angle < -1*pi/4 + 0.006 && angle > -1*pi/4 - 0.006){index = 0;}
                    else if(angle < -1*pi/8 + 0.006 && angle > -1*pi/8 - 0.006){index = 1;}
                    else if(angle < 0 + 0.006 && angle > 0 - 0.006){index = 2;}
                    else if(angle < 1*pi/8 + 0.006 && angle > 1*pi/8 - 0.006){index = 3;}
                    else if(angle < 1*pi/4 + 0.006 && angle > 1*pi/4 - 0.006){index = 4;}
                    else if(angle < -3*pi/4 + 0.006 && angle > -3*pi/4 - 0.006){index = 5;}
                    else if(angle < -7*pi/8 + 0.006 && angle > -7*pi/8 - 0.006){index = 6;}
                    else if(angle > pi - 0.006 || -angle > pi - 0.006){index = 7;}
                    else if(angle < 7*pi/8 + 0.006 && angle > 7*pi/8 - 0.006){index = 8;}
                    else if(angle < 3*pi/4 + 0.006 && angle > 3*pi/4 - 0.006){index = 9;}
                    else {continue;}
                    
                    if(distances.at(index) > distance){
                        distances.at(index) = distance;
                        detected_points.at(index) = contours.at(i).at(j);
                    };
                };
            }
        }
        
        for(size_t i = 0; i < detected_points.size(); i++){
            if(debug_mode){
                cv::drawMarker(frame, detected_points.at(i), color, cv::MARKER_CROSS, 7);
                std::cout << "Range " << i << " is " << distances.at(i) << "\n";
            }
        }
        
        if(debug_mode){
            //cv::drawMarker(frame, cv::Point(2,2), color, cv::MARKER_CROSS, 3);
            cv::imshow("Original Frame with Contours", frame);
            cv::waitKey(1);
        }
        
        /*
        currently using the front lasers only (NW, NNW, N, NNE, NE)
        */
//        sensor_msgs::LaserScan scan_msg;
//        ros::Time scan_time = ros::Time::now();
        sensor_msgs::msg::LaserScan scan_msg;
        rclcpp::Time scan_time = n_public->now();

        scan_msg.header.stamp = scan_time;
        scan_msg.header.frame_id = "laser_frame";
        scan_msg.angle_min = -1*pi/4;
        scan_msg.angle_max = 1*pi/4;
        scan_msg.angle_increment = 1*pi/8;
        scan_msg.time_increment = 0;
        scan_msg.range_min = 0.0;
        scan_msg.range_max = 2.0;
        scan_msg.ranges.assign(5, NAN);
        //TODO: delete line below
        //inverting the data order as the simulator used for training natively generates data in a counterclockwise fashion
        /*
        for(size_t i = 0; i < scan_msg.ranges.size(); i++){
            if(distances.at(4-i)) > 5.0){
                scan_msg.ranges[i] = distances.at(4-i);
            }
        }
        */
        double temp = 6.0;
        //NE
        temp = convert_NE(distances.at(4)) * 0.001;
        if(temp < 2.0 && temp > 0.0){
            scan_msg.ranges[0] = temp;
        }
        //NNE
        temp = convert_NNE(distances.at(3)) * 0.001;
        if(temp < 2.0 && temp > 0.0){
            scan_msg.ranges[1] = temp;
        }
        //N
        temp = convert_N(distances.at(2)) * 0.001;
        if(temp < 2.0 && temp > 0.0){
            scan_msg.ranges[2] = temp;
        }
        //NNW
        temp = convert_NNW(distances.at(1)) * 0.001;
        if(temp < 2.0 && temp > 0.0){
            scan_msg.ranges[3] = temp;
        }
        //NW
        temp = convert_NW(distances.at(0)) * 0.001;
        if(temp < 2.0 && temp > 0.0){
            scan_msg.ranges[4] = temp;
        }

//        laser_scan_pub.publish(scan_msg);
        laser_scan_pub->publish(scan_msg);        

        /*
        std::cout << "Real distance at NW is " << convert_NW(distances.at(0)) << ", pixel is " << distances.at(0) << "\n";
        std::cout << "Real distance at NNW is " << convert_NNW(distances.at(1)) << ", pixel is " << distances.at(1) << "\n";
        std::cout << "Real distance at N is " << convert_N(distances.at(2)) << ", pixel is " << distances.at(2) << "\n";
        std::cout << "Real distance at NNE is " << convert_NNE(distances.at(3)) << ", pixel is " << distances.at(3) << "\n";
        std::cout << "Real distance at NE is " << convert_NE(distances.at(4)) << ", pixel is " << distances.at(4) << "\n";
        std::cout << "Real distance at SW is " << convert_SW(distances.at(5)) << ", pixel is " << distances.at(5) << "\n";
        std::cout << "Real distance at SSW is " << convert_SSW(distances.at(6)) << ", pixel is " << distances.at(6) << "\n";
        std::cout << "Real distance at S is " << convert_S(distances.at(7)) << ", pixel is " << distances.at(7) << "\n";
        std::cout << "Real distance at SSE is " << convert_SSE(distances.at(8)) << ", pixel is " << distances.at(8) << "\n";
        std::cout << "Real distance at SE is " << convert_SE(distances.at(9)) << ", pixel is " << distances.at(9) << "\n";
        */
        
        if(debug_mode){
            cv::waitKey(0);
        }

//        ros::spinOnce();
        rclcpp::spin_some(n_public);
        rate.sleep();
    }

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    cv::destroyAllWindows();

    return 0;
}
	
	
	
