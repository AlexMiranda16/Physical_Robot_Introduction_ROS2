#include "object_detection.h"

const double pi = 3.14159265358979323846;

std::string sensor_img_path;

//contantes para dete��o de objetos
const int minThreshold = 240;
const int threshMultiplier = 3;
const int kernelSize = 3;
const int minRadiusFilter = 60;
const int binaryThreshold = 180;

//valores para a segmentação da imagem em regiões de interesse
//frente
const int N_beginningX = 90;
const int N_width = 420;
const int N_endingX = N_beginningX + N_width;
const int N_beginningY = 400;
const int N_height = 400;
//frente direita
const int NE_beginningX = 95;
const int NE_width = 470;
const int NE_endingX = NE_beginningX + NE_width;
const int NE_beginningY = 1;
const int NE_height = 400;

//frente esquerda
const int NW_beginningX = 90;
const int NW_width = 470;
const int NW_endingX = NW_beginningX + NW_width;
const int NW_beginningY = 800;
const int NW_height = 400;

//atrás
const int S_beginningX = 1200;
const int S_width = 300;
const int S_endingX = S_beginningX + S_width;
const int S_beginningY = 350;
const int S_height = 450;




//inicializa��es vari�veis de sensores
uint8_t N = 0, NE = 0, NW = 0, S = 0;
// N=Frente
// NE=Frente Direita
// NW=Frente Esquerda
// S=Tr�s
// SE=Tr�s Esquerda
// SW=Tr�s Direita

//fun��o que calcula o n�vel de per�go de objeto dependendo da sua dist�ncia ao robo. 0 min, 3 max
uint8_t levelDanger(float n)
{
    //cout << "distancia" << n << endl;
    
    if ((n <= 420) && (n > 200))
        return 1;

    else if ((n <= 200) && (n > 100))
        return 2;

    else if ((n <= 100) && (n >= 0))
        return 3;
    else
        return 0;

}



//filtra os objetos detetados, e identifica qual o mais pr�ximo, onde calcula o seu perigo
void sortDetectedObjects(std::string pos, cv::Mat img, std::vector<cv::Point2f> centers, std::vector<float> radius, std::vector<cv::Rect> rectangles)
{
    int height = img.rows;
    int width = img.cols;

    float distQuant = 0;
    float minDist = 1000;

    if (pos.compare("N") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {

            if ((float)radius[i] > minRadiusFilter) {

                distQuant = abs(rectangles[i].br().x - N_width);

                if (distQuant < minDist) {
                    minDist = distQuant;
                }

            }
        }

        N = levelDanger(minDist);

    }
    else if (pos.compare("S") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {
            if ((float)radius[i] > minRadiusFilter) {

                distQuant = abs(rectangles[i].tl().x - 0);

                if (distQuant < minDist) {
                    minDist = distQuant;
                }

            }
        }
        S = levelDanger(minDist);

    }
    else if (pos.compare("NE") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {
            if ((float)radius[i] > minRadiusFilter) {

                distQuant = sqrt(pow(abs(rectangles[i].br().x - NE_width), 2) + pow(abs(rectangles[i].tl().y - 400), 2));

                if (distQuant < minDist) {
                    minDist = distQuant;
                }

            }
        }

        NE = levelDanger(minDist);

    }
    else if (pos.compare("NW") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {
            if ((float)radius[i] > minRadiusFilter) {

                distQuant = sqrt(pow(abs(rectangles[i].br().x - NW_width), 2) + pow(abs(rectangles[i].br().y - 1), 2));

                if (distQuant < minDist) {
                    minDist = distQuant;
                }

            }
        }

        NW = levelDanger(minDist);

    }

}

//trabalho incacabado de elemina��o de circulos dentro de circulos
bool inRadious(cv::Point2f centerOrigin, cv::Point2f center2, int r) {

    float distBetweenCenters = sqrt(pow(abs((double)centerOrigin.x - center2.x), 2) + pow(abs((double)centerOrigin.y - center2.y), 2));

    if (distBetweenCenters <= r)
        return true;
    else
        return false;
}


//Encontrar objetos na imagem, e guardar os centros e os raios em vatores
void detectObj(cv::Mat img, int lowThreshold, int threshMult, int kernel_size, std::string pos)
{



    cv::Mat equalized, detected_edges;

    float sumX = 0, sumY = 0, sumR = 0;
    int num = 0;

    cv::Mat img_grey;
    //preprocessing on image
    //pyrMeanShiftFiltering(img, img, 10, 20);
    //medianBlur(img, img, 27);

    int erosion_size = 3;

   /* Mat element = getStructuringElement(MORPH_ELLIPSE,
        Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        Point(erosion_size, erosion_size));

    morphologyEx(img, img,
        MORPH_GRADIENT, element,
        Point(-1, -1), 1);*/


    cvtColor(img, img_grey, cv::COLOR_BGR2GRAY);
    blur(img_grey, img_grey, cv::Size(5, 5));

    equalizeHist(img_grey, equalized);

    

    threshold(equalized, equalized, binaryThreshold, 255, 3);

    /*cv::imshow("Image ", equalized);
    cv::waitKey(0);*/

    /*cv::imshow("Image ", equalized);
    cv::waitKey(0);*/


    //Canny(equalized, detected_edges, lowThreshold, lowThreshold * threshMult, kernel_size);

    std::vector<std::vector<cv::Point> > contours;
    findContours(equalized, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point> > contours_poly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    std::vector<cv::Point2f> centers(contours.size());
    std::vector<float> radius(contours.size());
    for (size_t j = 0; j < contours.size(); j++)
    {
        approxPolyDP(contours[j], contours_poly[j], 3, true);
        boundRect[j] = boundingRect(contours_poly[j]);
        minEnclosingCircle(contours_poly[j], centers[j], radius[j]);
    }
    //Mat drawing = Mat::zeros(detected_edges.size(), CV_8UC3);

    //eliminar deteção de objetos sobrepostas
    int i = 0;
    for (size_t i = 0; i < contours.size(); i++) {

        //sumX = centers[i].x;
        //sumY = centers[i].y;
       // sumR = radius[i];
        num = 1;

        for (size_t j = 0; j < contours.size(); j++)
        {
            if ((i != j)) {

                if ((radius[j] != 0) && (inRadious(centers[i], centers[j], radius[i]))) {
                    /*sumX += centers[j].x;
                    sumY += centers[j].y;
                    sumR += radius[j];
                    radius[j] = 0;
                    num += 1;*/
                    if (radius[j] > radius[i])
                        radius[i] = 0;
                    else if (radius[j] <= radius[i])
                        radius[j] = 0;
                    
                }
            }
        }

    }


    //debug only. desenha contornos e circulos nos objetos detetados
     for (size_t j = 0; j < contours.size(); j++)
     {
         cv::Scalar color = cv::Scalar(0, 0, 255);

         if ((int)radius[j] > minRadiusFilter) {
             rectangle(img, boundRect[j].tl(), boundRect[j].br(), color, 2);
             circle(img, centers[j], (int)radius[j], color, 2);
         }
     }

    sortDetectedObjects(pos, img, centers, radius, boundRect);

}


//fun��o que pega em cada frame do video, aplica pre processamento, segmenta a imagem e deteta os objetos em cada segmento da imagem
std::vector<uint8_t> frameProcessing(cv::Mat frame) {

    N = 0;
    NW = 0;
    NE = 0;
    S = 0;

    //divisão dos frames em regiões de interece
    //ROI(pixelHorizontalInicio, PixelVerticalInicio, QuantidadePixeisHorizontal, QuantidadePixeisVertical)
    cv::Rect ROI_NE(NE_beginningX, NE_beginningY, NE_width, NE_height);
    cv::Mat image_NE = frame(ROI_NE);

    cv::Rect ROI_N(N_beginningX, N_beginningY, N_width, N_height);
    cv::Mat image_N = frame(ROI_N);

    cv::Rect ROI_NW(NW_beginningX, NW_beginningY, NW_width, NW_height);
    cv::Mat image_NW = frame(ROI_NW);

    cv::Rect ROI_S(S_beginningX, S_beginningY, S_width, S_height);
    cv::Mat image_S = frame(ROI_S);

    //deteta os objetos com os thresholds, e com o identificador de que parte �.
    detectObj(image_NE, minThreshold, threshMultiplier, kernelSize, "NE");
    detectObj(image_N, minThreshold, threshMultiplier, kernelSize, "N");
    detectObj(image_NW, minThreshold, threshMultiplier, kernelSize, "NW");
    detectObj(image_S, minThreshold, threshMultiplier, kernelSize, "S");

    /*//imprimir o valor dos sensores. 
    cout << "FD: " << NE << endl;
    cout << "FE: " << NW << endl;
    cout << "F: " << N << endl;
    cout << "T: " << S << endl;
    cout << "__________________________________________________________" << endl;

    return frame;*/

    //vetor para retornar valores dos sensores
    std::vector<uint8_t> retorna_valores{NE , N , S, NW};
    return retorna_valores;

}

std::string find_sensorIMG_path(char sensor , int valor_sensor){
    std::string path = sensor_img_path;

    switch (sensor)
    {
    case 'f': //frente
        path.append("frente");
        break;

    case 't': //trás
        path.append("tras");
        break;

    case 'd': //direita
        path.append("direita");
        break;

    case 'e': //esquerda
        path.append("esquerda");
        break;
    }

    switch (valor_sensor)
    {
    case 1:
        path.append("1.png");
        break;
    case 2:
        path.append("2.png");
        break;
    case 3:
        path.append("3.png");
        break;
    case 4:
        path.append("4.png");
        break;

    }
    //ROS_INFO_STREAM("file_path: " << path);
    //std::cout << "path is " << path << "\n";
    return path;
}


cv::Mat place_sensor(cv::Mat frame, cv::Mat sensor, int position_x, int position_y){

    cv::Mat mask;
    cv::Mat rgbLayer[4];
    cv::split(sensor,rgbLayer);

    if(sensor.channels() == 4)
    {
        split(sensor,rgbLayer);         // seperate channels
        cv::Mat cs[3] = { rgbLayer[0],rgbLayer[1],rgbLayer[2] };
        merge(cs,3,sensor);  // glue together again
        mask = rgbLayer[3];       // png's alpha channel used as mask
    }

    // Get the destination ROI (and make sure it is within the image)
    cv::Rect dstRC = cv::Rect(position_x, position_y, sensor.size().width, sensor.size().height);
    cv::Mat dstROI = frame(dstRC);
    // Copy the pixels from src to dst.
    sensor.copyTo(dstROI,mask);

    return frame;
}

cv::Mat process_frame(cv::Mat frame, std::vector<uint8_t> sensor_valores)
{

    for(;;)
    {

        // Import sensor images
        cv::Mat sensor_front = cv::imread(find_sensorIMG_path('f',sensor_valores[1]+1), cv::IMREAD_UNCHANGED);
        cv::Mat sensor_back = cv::imread(find_sensorIMG_path('t',sensor_valores[2]+1), cv::IMREAD_UNCHANGED);
        cv::Mat sensor_left = cv::imread(find_sensorIMG_path('e',sensor_valores[3]+1), cv::IMREAD_UNCHANGED);
        cv::Mat sensor_right = cv::imread(find_sensorIMG_path('d',sensor_valores[0]+1), cv::IMREAD_UNCHANGED);

        //Resize sensor images
        cv::resize(sensor_front,sensor_front,cv::Size(60,37));
        cv::resize(sensor_back,sensor_back,cv::Size(60,37));
        cv::resize(sensor_left,sensor_left,cv::Size(60,71));
        cv::resize(sensor_right,sensor_right,cv::Size(60,71));


        int cx = (frame.cols - sensor_front.size().width) / 2;
        if (!sensor_front.empty()) {
            frame = place_sensor(frame, sensor_front, cx, frame.rows/7);
        }
        if (!sensor_back.empty()) {
            frame = place_sensor(frame, sensor_back, cx, 5.2*frame.rows/7);

        }
        if (!sensor_left.empty()) {
            frame = place_sensor(frame, sensor_left, 7*cx/12, frame.rows/6);
        }
        if (!sensor_right.empty()) {
            frame = place_sensor(frame, sensor_right, 17*cx/12, frame.rows/6);
        }
        return frame;
    }

}

int main(int argc, char** argv)
{
//    ros::init(argc, argv, "object_detection_node");
//    ros::NodeHandle n_public("~");
//    ros::Rate rate(20); //rate set to 20Hz
    rclcpp::init(argc, argv);
    auto n_public = rclcpp::Node::make_shared("object_detecion");
    rclcpp::Rate rate(20); //rate set to 20Hz

    bool debug_mode = false;
//    n_public.getParam("debug", debug_mode);
    n_public->get_parameter("debug", debug_mode);
    bool record_mode = false;
//    n_public.getParam("record", record_mode);
//    n_public.getParam("sensor_img_path", sensor_img_path);
    n_public->get_parameter("record", record_mode);
    n_public->get_parameter("sensor_img_path", sensor_img_path);
    std::string recording_path;
//    n_public.getParam("recording_path", recording_path);
    n_public->get_parameter("recording_path", recording_path);


//    ros::Publisher object_detection_pub = n_public.advertise<fisheye_camera_simulated_sensors_ros1::ObjectDetection>("/object_detection", 1);
    rclcpp::Publisher<custom_message::msg::ObjectDetection>::SharedPtr object_detection_pub;
    object_detection_pub = n_public->create_publisher<custom_message::msg::ObjectDetection>("/object_detection", 1);

    /*
    // Create a VideoCapture object and open the input file
    // If the input is the web camera, pass 0 instead of the video file name
    cv::VideoCapture cap(0);

    // Check if camera opened successfully
    if(!cap.isOpened())
    {
        ROS_WARN_STREAM("Error opening video stream or file");
        return -1;
    }
    */

    raspicam::RaspiCam_Cv Camera;
    Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    Camera.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    Camera.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
    Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);
    //shutter speed, 1-100 for 0-33ms
    //Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);


    std::cout << "Opening Camera..." << std::endl;
    if (!Camera.open())
    {
        std::cerr << "Error opening the camera" << std::endl;
        return -1;
    }

    cv::VideoWriter video(recording_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(462, 500));
    
//    ros::Duration(1.0).sleep();

    while(rclcpp::ok())//ros::ok())
    {
        cv::Mat frame;
        /*
        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty()) break;
        */

        // Get new frame
        Camera.grab();
        Camera.retrieve(frame);

        cv::Mat cropped_image = frame(cv::Rect(100, 0, 1300, 1200));

        // Resize image to 800x450 (to publish to the GUI)
        cv::Mat resized_frame;
        cv::resize(cropped_image, resized_frame, cv::Size(500, 462));

        //Rotate frame 90º
        cv::rotate(resized_frame, resized_frame, cv::ROTATE_90_CLOCKWISE);
        //get sensor values
        std::vector<uint8_t> sensor_values = frameProcessing(frame);
        resized_frame=process_frame(resized_frame, sensor_values);

        custom_message::msg::ObjectDetection obj_det_msg;
        obj_det_msg.left = sensor_values[3];
        obj_det_msg.right = sensor_values[0];
        obj_det_msg.back = sensor_values[2];
        obj_det_msg.front = sensor_values[1];

        object_detection_pub->publish(obj_det_msg);

        if(record_mode){
            //cv2.cvtColor(resized_frame,cv2.COLOR_RGB2BGR);
            video.write(resized_frame);
        }

        if(debug_mode){
            cv::imshow("Original Frame", frame);
            cv::waitKey(1);

            std::cout << "Resized frame size is: " << resized_frame.size() << "\n";

            cv::imshow("Frame with Detection", resized_frame);
            cv::waitKey(0);
        }

//        ros::spinOnce();
        rclcpp::spin_some(n_public);
        rate.sleep();
    }

    // When everything done, release the video capture object
    Camera.release();
    video.release();

    // Closes all the frames
    cv::destroyAllWindows();

    std::cout << "uwu\n";

    return 0;

}
