#include "calibration_utils.h"


const double pi = 3.14159265358979323846;

int min_dist = 0;
int max_dist = 240;
cv::Mat base_frame;
std::vector<double> distances;
std::vector<cv::Point> detected_points;

void dist_callback(int, void* ){
    cv::Mat frame = base_frame.clone();
    cv::Scalar color = cv::Scalar(0,0,255);

    for(size_t i = 0; i < detected_points.size(); i++){
        
        if(distances.at(i) > min_dist && distances.at(i) < max_dist){
            cv::drawMarker(frame, detected_points.at(i), color, cv::MARKER_CROSS, 7);
            std::cout << "Range " << i << " is " << distances.at(i) << "\n";
            
            /*
            else{
                cv::drawMarker(frame, detected_points.at(i), color, cv::MARKER_CROSS, 7);
            }
            */
            
        }
    }
        //cv::drawMarker(frame, cv::Point(2,2), color, cv::MARKER_CROSS, 3);
    cv::imshow("Original Frame with Contours", frame);

}

int main(int argc, char** argv)
{
//    ros::init(argc, argv, "camera_calibration_node");
//    ros::NodeHandle n_public("~");
    rclcpp::init(argc, argv);
    auto n_public = rclcpp::Node::make_shared("camera_calibration_node");

    bool debug_mode = false;
//    n_public.getParam("debug", debug_mode);
    n_public->get_parameter("debug",debug_mode);

    //std::cout << "Debug mode is set to " << debug_mode << "\n";
    std::string image_path;
//    n_public.getParam("image_path", image_path);
    n_public->get_parameter("image_path", image_path);

    // Create a VideoCapture object and open the input file
    // If the input is the web camera, pass 0 instead of the video file name
    cv::VideoCapture cap(image_path);

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

        base_frame = frame;

        // If the frame is empty, break immediately
        if (frame.empty()) break;

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
        /*
        cv::Mat img_bilat;
        cv::bilateralFilter(img_gray, img_bilat, 11, 75, 75);
        if(debug_mode){
            cv::imshow("Bilat Frame", img_bilat);
            cv::waitKey(1);
        }
        */
        cv::Mat img_canny;
        cv::Canny(img_gray, img_canny, 60, 60, 3);
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
        
        //std::vector<double> distances;
        //std::vector<cv::Point> detected_points;
        for(size_t i = 0; i < contours.size(); i++){
            //drawContours(frame, contours, (int) i, color);
            //std::cout << "Contour " << i << " is " << contours.at(i);
            for(size_t j = 0; j < contours.at(i).size(); j++){

                //relative positions of the points to the center of the image
                int x_diff = contours.at(i).at(j).x - frame_width/2;
                int y_diff = contours.at(i).at(j).y - frame_height/2;

                //distance of the point to the center of the image
                double distance = cv::sqrt(x_diff*x_diff + y_diff*y_diff);

                //angle of the vector (Point-Center) relative to vector (-1, 0)
                double angle = acos((x_diff * -1 + y_diff * 0)/(distance * 1));
                if(y_diff > 0){angle = -angle;};

                if(angle < pi + 0.006 && angle > pi - 0.006){
                    detected_points.push_back(contours.at(i).at(j));
                    distances.push_back(distance);
                }
            }
        }
        const char* source_window = "Funky window";
        cv::namedWindow( source_window );
        cv::imshow( source_window, img_canny );
        const int limit_dist = 250;
        cv::createTrackbar( "Min dist:", source_window, &min_dist, limit_dist, dist_callback );
        cv::createTrackbar( "Max dist:", source_window, &max_dist, limit_dist, dist_callback );
        
        /*
        for(size_t i = 0; i < detected_points.size(); i++){
            if(debug_mode){
                if(distances.at(i) > 242 && distances.at(i) < 244){
                    cv::drawMarker(frame, detected_points.at(i), color, cv::MARKER_CROSS, 7);
                }
                
                else{
                    cv::drawMarker(frame, detected_points.at(i), color, cv::MARKER_CROSS, 7);
                }
                
                std::cout << "Range " << i << " is " << distances.at(i) << "\n";
            }
        }
        if(debug_mode){
            //cv::drawMarker(frame, cv::Point(2,2), color, cv::MARKER_CROSS, 3);
            cv::imshow("Original Frame with Contours", frame);
            cv::waitKey(1);
        }
        */

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

        if(debug_mode){
            cv::waitKey(0);
        }

//        ros::spinOnce();
        rclcpp::spin_some(n_public);
    }

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    cv::destroyAllWindows();

    return 0;
}