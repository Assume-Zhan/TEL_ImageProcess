#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <map>

#include "block_detector/ObjectPt_srv.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#define MAX_COL_CAM 630

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

    void holdOnFunc();

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


    std::pair<cv::Scalar, cv::Scalar> dark_hsv_min_max_; // HSV {min, max}
    std::pair<cv::Scalar, cv::Scalar> light_hsv_min_max_;

    std::pair<double, double> cam_world_x[MAX_COL_CAM];
    std::pair<double, double> cam_world_y[MAX_COL_CAM];

    // DEBUGMODE : debug image
    cv::Mat debug_image;

    // Parameter get by ros_param
    bool debugMode_ = false;

    int cam_index_ = 2;
    double min_area_size_ = 100;
    double contour_epsilon_mul_ = 0.02;
    int captured_times_ = 1;

    std::string tf_file_name_ = "/home/assume/Desktop/TLE_Navigation/src/block_detector/transform/coordinate_tf.csv";

    int dark_h_min_ = 98;
    int dark_s_min_ = 62;
    int dark_v_min_ = 105;
    int dark_h_max_ = 132;
    int dark_s_max_ = 255;
    int dark_v_max_ = 255;

    int light_h_min_ = 78;
    int light_s_min_ = 64;
    int light_v_min_ = 120;
    int light_h_max_ = 96;
    int light_s_max_ = 198;
    int light_v_max_ = 255;

    int threshold_canny_1_ = 15;
    int threshold_canny_2_ = 10;

    int threshold_min_ = 50;
    int threshold_max_ = 255;
};