#include <ros/ros.h>
#include "block_detector.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "block_detector_node");

    ros::NodeHandle nh_("block_detector_node");

    BlockDetector bdr(nh_);

}