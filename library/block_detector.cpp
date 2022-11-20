#include "block_detector.h"

BlockDetector::BlockDetector(ros::NodeHandle& nh): nh_(nh){

    /* Service advertise */
    srv_camera_state_ = nh.advertiseService("getObjectPts", &BlockDetector::camera_srv_cb, this);

    this->dark_hsv_min_max_ = {cv::Scalar(98, 62, 105), cv::Scalar(132, 255, 255)};
    this->light_hsv_min_max_ = {cv::Scalar(78, 64, 120), cv::Scalar(96, 198, 255)};
    this->get_tf_points(FILE_NAME_CAM_TF);
}


bool BlockDetector::camera_srv_cb(bdr_srv::Request& req, bdr_srv::Response& res){
    // Open the camera
    cv::VideoCapture camera(0); // TODO : modularize the camera index
    cv::Mat frame;


    for(int times = 0; times < this->captured_times; times++){
        bool ret = camera.read(frame);
        std::map<char, cv::Point2d> block_point_world;
        block_point_world = this->transformation(this->blocks_catch(this->preprocess(frame)));
        ROS_INFO_STREAM("Time " << times << ":");
        for(auto x : block_point_world){
            ROS_INFO_STREAM("   Block : " << x.first << " , at : " << x.second);
        }
    }


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
    // Output canny
    cv::cvtColor(filtered, filtered, cv::COLOR_BGR2GRAY);
    cv::Mat canny;
    cv::Canny(filtered, canny, 15, 10);

    // Threshold
    cv::Mat thresholded;
    cv::threshold(canny, thresholded, 50, 255, cv::THRESH_BINARY);

    return thresholded;

}


std::multimap<char, cv::Point2f> BlockDetector::blocks_catch(cv::Mat preprocessed_img){

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(preprocessed_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // Simplified the contours
    std::vector<std::vector<cv::Point>> contours_simp;
    for(size_t i = 0; i < contours.size(); i++){

        // Filter some small area
        double area = cv::contourArea(contours[i]);
        if(area <= 100) continue; // TODO : modularize the area size

        // Use arc length to set proper epsilon
        double arcLen = cv::arcLength(contours[i], true);

        // Simplify the contours
        std::vector<cv::Point> better_contour;
        cv::approxPolyDP(cv::Mat(contours[i]), better_contour, arcLen * 0.02, true);
        // TODO : modularize the 55 for epsilon

        // convex hull
        // cv::convexHull(cv::Mat(contours[i]), better_contour);
        // std::vector<cv::Point> better_contour_;
        // cv::approxPolyDP(cv::Mat(better_contour), better_contour_, arcLen * 0.02, true);
        // ROS_INFO_STREAM("info" << better_contour_.size());
        contours_simp.push_back(better_contour);
    }

    // DEBUGMODE : draw contours
    cv::Mat dp_image = cv::Mat::zeros(preprocessed_img.size(), CV_8UC3);
    drawContours(dp_image, contours_simp, -2, cv::Scalar(255, 0, 255), 1, 0);

    // Find the block and calculate the center of the block
    std::multimap <char, cv::Point2f> block_positions;
    cv::RotatedRect box;
    cv::Point2f vertices[4];

    for(int idx = 0; idx < contours_simp.size(); idx++){
        if(contours_simp[idx].size() == 6){  /* L block */
            box = minAreaRect(contours_simp[idx]);
            box.points(vertices);

            cv::Point2f center;
            for(auto x : vertices){
                center += x;
            }

            block_positions.insert({'L', center / 4});
            circle(dp_image, center / 4, 0, cv::Scalar(0, 255, 255), 8);
        }
        else if(contours_simp[idx].size() == 8){  /* T block */
            box = minAreaRect(contours_simp[idx]);
            box.points(vertices);

            cv::Point2f center;
            for(auto x : vertices){
                center += x;
            }

            block_positions.insert({'T', center / 4});
            circle(dp_image, center / 4, 0, cv::Scalar(0, 255, 255), 8);
        }
    }

    return block_positions;
}


std::map<char, cv::Point2d> BlockDetector::transformation(std::multimap<char, cv::Point2f> blocks){
    std::map<char, cv::Point2d> Positions;

    for(auto block : blocks){
        // Set goal x y
        double goal_x = block.second.x, goal_y = block.second.y;

        // Find the nearest point
        double proper_x, proper_y, min_err = -1;
        for(int idx = 0; idx < MAX_COL_CAM; idx++){
            double err = abs(cam_world_x[idx]._cam_ - goal_x) + abs(cam_world_y[idx]._cam_ - goal_y);

            if(min_err == -1 || err < min_err){
                proper_x = cam_world_x[idx]._world_;
                proper_y = cam_world_y[idx]._world_;
                min_err = err;
            }
        }

        if(Positions.find(block.first) == Positions.end()){
            Positions.insert({block.first, cv::Point2d{proper_x, proper_y}});
        }
        else if(Positions[block.first].y < proper_y){
            Positions.insert({block.first, cv::Point2d{proper_x, proper_y}});
        }
    }

    return Positions;
}


void BlockDetector::get_tf_points(std::string filePath){
    std::ifstream tf_file(filePath);
    if(!tf_file.is_open()){
        std::cout << "Unable to open " << filePath << '\n';
    }

    // 2) Read data by file stream
    double x_world, y_world, x_cam, y_cam;
    std::string column;
    int idx = 0;
    while(std::getline(tf_file, column)){
        std::stringstream sstring(column);
        sstring >> x_world >> y_world >> x_cam >> y_cam;
        cam_world_x[idx]._cam_ = x_cam;
        cam_world_x[idx]._world_ = x_world;
        cam_world_y[idx]._cam_ = y_cam;
        cam_world_y[idx]._world_ = y_world;
        idx++;
    }
}


void BlockDetector::debug(){
    cv::VideoCapture camera(0);
    cv::Mat frame;

    while(true){
        bool ret = camera.read(frame);
        cv::imshow("origin_frame", frame);

        // Preprocess the frame
        frame = this->preprocess(frame);
        std::multimap<char, cv::Point2f> block_point_cam = this->blocks_catch(frame);

        cv::imshow("frame", frame);

        char event = cv::waitKey(1);
        if(event == 'w'){
            std::map<char, cv::Point2d> block_point_world = this->transformation(block_point_cam);
            for(auto x : block_point_world){
                ROS_INFO_STREAM("Block : " << x.first << " , at : " << x.second);
            }
        }
        else if(event == 'q') break;

    }
}