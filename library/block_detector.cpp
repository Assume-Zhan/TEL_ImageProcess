#include "block_detector.h"

BlockDetector::BlockDetector(ros::NodeHandle& nh): nh_(nh){

    /* Service advertise */
    srv_camera_state_ = nh.advertiseService("getObjectPts", &BlockDetector::camera_srv_cb, this);

    this->dark_hsv_min_max_ = {cv::Scalar(98, 62, 105), cv::Scalar(132, 255, 255)};
    this->light_hsv_min_max_ = {cv::Scalar(94, 79, 81), cv::Scalar(108, 255, 245)};
}

bool BlockDetector::camera_srv_cb(bdr_srv::Request& req, bdr_srv::Response& res){
    return true;
}

cv::Mat BlockDetector::preprocess(cv::Mat frame){
    // preprocess the raw frame from camera 

    // Gaussian blur
    // TODO : understand what the 3 and 0 means
    cv::GaussianBlur(frame, frame, cv::Size(9, 9), 3, 0, cv::BORDER_REFLECT);

    // Covert color from BGR to HSV
    cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);

    // Filter unnecessary color
    cv::Mat maskDark, maskLight, mask;
    cv::Mat filtered = cv::Mat::zeros(frame.size(), CV_8UC3);

    // Get dark and light mask
    cv::inRange(frame, this->dark_hsv_min_max_.first, this->dark_hsv_min_max_.second, maskDark);
    cv::inRange(frame, this->light_hsv_min_max_.first, this->light_hsv_min_max_.second, maskLight);

    // Get the highest mask and mask the frame
    cv::bitwise_or(maskDark, maskLight, mask);
    cv::bitwise_and(frame, frame, filtered, mask); // Result must be defined all zero

    // Find the bolder of the result
    cv::Mat canny;
    cv::Canny(filtered, canny, 15, 10);

    return canny;

}

std::map <char, std::pair<int, int>> BlockDetector::blocks_catch(cv::Mat preprocessed){

}

std::map <char, std::pair<int, int>> BlockDetector::transformation(std::map <char, std::pair<int, int>> blocks){

}

void BlockDetector::debug(){
    cv::VideoCapture camera(0);
    cv::Mat frame;

    while(true){
        bool ret = camera.read(frame);

        // Preprocess the frame
        frame = this->preprocess(frame);

        cv::imshow("frame", frame);

        if(cv::waitKey(1) == 'q') break;
    }
}