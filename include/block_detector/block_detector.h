#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <map>

#include "block_detector/ObjectPt_srv.h"

#define MAX_COL_CAM 546
#define FILE_NAME_CAM_TF "/home/assume/Desktop/TLE_Navigation/\
src/block_detector/transform/coordinate_tf.csv"

#define _cam_ first
#define _world_ second

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
     * @return std::multimap <char, std::pair<int, int>>
     */
    std::multimap<char, cv::Point2f> blocks_catch(cv::Mat preprocessed);

    /**
     * @brief Transform frame points to world points
     *
     * @param blocks Frame point of blocks
     * @return std::map <char, std::pair<int, int>>
     */
    std::map<char, cv::Point2d> transformation(std::multimap<char, cv::Point2f> blocks);

    /**
     * @brief Get the tf points object
     *
     * @param filename
     */
    void get_tf_points(std::string filename);

    ros::NodeHandle nh_;
    ros::ServiceServer srv_camera_state_;

    int captured_times = 1;

    std::pair<cv::Scalar, cv::Scalar> dark_hsv_min_max_; // HSV {min, max}
    std::pair<cv::Scalar, cv::Scalar> light_hsv_min_max_;

    std::pair<double, double> cam_world_x[MAX_COL_CAM];
    std::pair<double, double> cam_world_y[MAX_COL_CAM];
};