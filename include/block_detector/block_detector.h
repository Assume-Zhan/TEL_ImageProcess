#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <map>

#include "block_detector/ObjectPt_srv.h"

using bdr_srv = block_detector::ObjectPt_srv;

class BlockDetector{

public:
    BlockDetector(ros::NodeHandle& nh);

    ~BlockDetector(){}

    /**
     * @brief Just for debugging
     *
     */
    void debug();

private:

    /**
     * @brief service call back
     *
     * @param req from client
     * @param res to client
     * @return bool
     */
    bool camera_srv_cb(bdr_srv::Request& req, bdr_srv::Response& res);

    /**
     * @brief For preprocess a raw image
     *
     * @param frame raw image
     * @return cv::Mat
     */
    cv::Mat preprocess(cv::Mat frame);

    /**
     * @brief Detect the blocks
     *
     * @param preprocessed preprocessed image
     * @return std::map <char, std::pair<int, int>>
     */
    std::map <char, std::pair<int, int>> blocks_catch(cv::Mat preprocessed);

    /**
     * @brief Transform frame points to world points
     *
     * @param blocks Frame point of blocks
     * @return std::map <char, std::pair<int, int>>
     */
    std::map <char, std::pair<int, int>> transformation(std::map <char, std::pair<int, int>> blocks);


    ros::NodeHandle nh_;
    ros::ServiceServer srv_camera_state_;

    std::pair<cv::Scalar, cv::Scalar> dark_hsv_min_max_; // HSV {min, max}
    std::pair<cv::Scalar, cv::Scalar> light_hsv_min_max_;
};