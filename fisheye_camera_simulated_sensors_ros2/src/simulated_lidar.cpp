#include "simulated_lidar.h"

int main(int argc, char** argv)
{
//    ros::init(argc, argv, "simulated_lidar_node");
//    ros::NodeHandle n_public("~");
//    ros::Rate rate(20); //rate set to 20Hz
    rclcpp::init(argc, argv);
    auto n_public = rclcpp::Node::make_shared("simulated_lidar_node");
    rclcpp::Rate rate(20); //rate set to 20Hz

    bool debug_mode = false;
//    n_public.getParam("debug", debug_mode);
    n_public->get_parameter("debug", debug_mode);
    //std::cout << "Debug mode is set to " << debug_mode << "\n";

//    ros::Publisher laser_scan_pub = n_public.advertise<sensor_msgs::LaserScan>("/static_laser", 1);
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub;
    laser_scan_pub = n_public->create_publisher<sensor_msgs::msg::LaserScan>("/static_laser", 1);

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

//        sensor_msgs::LaserScan scan_msg;
        sensor_msgs::msg::LaserScan scan_msg;
//        ros::Time scan_time = ros::Time::now();       
        rclcpp::Time scan_time = n_public->now();

        scan_msg.header.stamp = scan_time;
        scan_msg.header.frame_id = "laser_frame";
        scan_msg.angle_min = -3.14159265359;
        scan_msg.angle_max = 3.14159265359;
        scan_msg.angle_increment = 0.06981317007;
        scan_msg.time_increment = 0;
        scan_msg.range_min = 135.0;
        scan_msg.range_max = 220.0;
        //scan_msg.set_ranges_size(91);
        scan_msg.ranges.assign(91, std::numeric_limits<double>::infinity());

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

        /*
        cv::Mat img_with_circles = img_canny;
        cv::circle(img_with_circles, cv::Point(frame_width / 2, frame_height / 2), 135, cv::Scalar(255,0,0));
        cv::circle(img_with_circles, cv::Point(frame_width / 2, frame_height / 2), 220, cv::Scalar(255,0,0));
        cv::imshow("Circled Frame", img_with_circles);
        cv::waitKey(1);
        */

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(img_canny, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
        cv::Scalar color = cv::Scalar(0,0,255);
        double min_angle = -3.14159265359;
        double max_angle = 3.14159265359;
        double increment = 0.06981317007;
        std::vector<double> distances(91, 300);
        std::vector<cv::Point> detected_points(91, cv::Point(0,0));
        for(size_t i = 0; i < contours.size(); i++){
            //drawContours(frame, contours, (int) i, color);
            //std::cout << "Contour " << i << " is " << contours.at(i);
            for(size_t j = 0; j < contours.at(i).size(); j++){
                int x_diff = contours.at(i).at(j).x - frame_width/2;
                int y_diff = contours.at(i).at(j).y - frame_height/2;
                double distance = cv::sqrt(x_diff*x_diff + y_diff*y_diff);
                double angle = acos((x_diff * -1 + y_diff * 0)/(distance * 1));
                if(angle > max_angle){continue;};
                if(y_diff > 0){angle = -angle;};
                if(distance < 220.0 && distance > 135.0){
                    //cv::drawMarker(frame, contours.at(i).at(j), color, cv::MARKER_CROSS, 3);
                    //std::cout << "Point at " << contours.at(i).at(j) << " with distance " << distance << " and angle " << angle;
                    int index = std::round((angle - min_angle)/increment);
                    //std::cout << "Index " << index << " for point " << contours.at(i).at(j) << "\n";
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
            if(distances.at(i) < 250.0){
                scan_msg.ranges[i] = distances.at(i);
            }
        }
        if(debug_mode){
            //cv::drawMarker(frame, cv::Point(2,2), color, cv::MARKER_CROSS, 3);
            cv::imshow("Original Frame with Contours", frame);
            cv::waitKey(1);
        }
        

        /*
        cv::Mat kernel = getStructuringElement( cv::MORPH_RECT, cv::Size( 5, 5 ), cv::Point( 2, 2 ) );
        
        cv::Mat img_dilate;
        cv::dilate(img_canny, img_dilate, kernel , cv::Point(-1,-1) ,7);
        cv::imshow("Dilate Frame", img_dilate);
        cv::waitKey(1);

        cv::Mat img_erode;
        cv::erode(img_canny, img_erode, kernel , cv::Point(-1,-1) ,7);
        cv::imshow("Erode Frame", img_erode);
        cv::waitKey(1);
        */

//        laser_scan_pub.publish(scan_msg);
        laser_scan_pub->publish(scan_msg);

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