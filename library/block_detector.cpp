#include "block_detector.h"

BlockDetector::BlockDetector(ros::NodeHandle& nh): nh_(nh){

    /* Service advertise */
    srv_camera_state_ = nh.advertiseService("getObjectPts", &BlockDetector::camera_srv_cb, this);


}

bool BlockDetector::camera_srv_cb(bdr_srv::Request& req, bdr_srv::Response& res){
    return true;
}

cv::Mat BlockDetector::preprocess(cv::Mat frame){

}

std::map <char, std::pair<int, int>> BlockDetector::blocks_catch(cv::Mat preprocessed){

}

std::map <char, std::pair<int, int>> BlockDetector::transformation(std::map <char, std::pair<int, int>> blocks){

}