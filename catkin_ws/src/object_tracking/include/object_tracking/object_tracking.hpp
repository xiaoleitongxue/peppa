//
// Created by lilei on 7/24/22.
//

#ifndef OBJECT_TRACKING_OBJECT_TRACKING_H
#define OBJECT_TRACKING_OBJECT_TRACKING_H
// c++
#include <pthread.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// ROS
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/tracking.hpp>

// Darknet_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h>
using namespace cv;
typedef struct {
    float x, y, w, h, prob;
    int num, Class;
}RosBox_;

typedef struct {
    cv::Mat image;
    std_msgs::Header header;
}CvMatWithHeader_;

class ObjectTracking{
public:
    explicit ObjectTracking(ros::NodeHandle nh);
    ~ObjectTracking();
private:
    void init();
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
    void boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes& msg);
    void objectCountCallback(const darknet_ros_msgs::ObjectCount& msg);
    void detectionImageCallback(const sensor_msgs::ImageConstPtr& msg);
    ros::NodeHandle nodeHandle_;
    // Advertise and subscribe to image opics
    image_transport::ImageTransport imageTransport_;
    // ROS subscriber and publisher
    image_transport::Subscriber imageSubscriber_;
    ros::Subscriber objectSubscriber_;
    ros::Subscriber boundingBoxesSubscriber_;
    ros::Subscriber detectionImageSubscriber_;
    // storage received image
    cv::Mat camImageCopy_;
    darknet_ros_msgs::BoundingBoxes boundingBoxesMsg_;
    darknet_ros_msgs::ObjectCount objectCountMsg_;
    int objectCount_;
    cv::Mat detectionImageCopy_;
    std::vector<Rect> roiBoxes_;
    void* trackingInThread();
    Ptr<MultiTracker> multiTracker_;
    void tracking();
};
#endif //OBJECT_TRACKING_OBJECT_TRACKING_H
