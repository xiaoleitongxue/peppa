//
// Created by lilei on 7/24/22.
//
#include <ros/ros.h>
#include "object_tracking/object_tracking.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "object_tracking");
    ros:: NodeHandle nodeHandle("~");
    ObjectTracking objectTracking(nodeHandle);
    ros::spin();
    
    return 0;
}