//
// Created by lilei on 7/24/22.
//
#include "object_tracking/object_tracking.hpp"
#include <string>
#include <opencv2/tracking.hpp>
ObjectTracking::ObjectTracking(ros::NodeHandle nh) : nodeHandle_(nh), imageTransport_(nodeHandle_)
{
    init();
}

ObjectTracking::~ObjectTracking()
{
}

void ObjectTracking::init()
{
    std::string cameraTopicName = "/camera/rgb/image_raw";
    std::string objectDetectionName = "/darknet_ros/found_object";
    std::string boundingBoxesTopicName = "/darknet_ros/bounding_boxes";
    std::string detectionImageTopicName = "/darknet_ros/detection_image";
    ROS_INFO("[Object Tracking] init().");
    imageSubscriber_ = imageTransport_.subscribe(cameraTopicName, 1, &ObjectTracking::cameraCallback, this);
    objectSubscriber_ = nodeHandle_.subscribe(objectDetectionName, 1, &ObjectTracking::objectCountCallback, this);
    boundingBoxesSubscriber_ = nodeHandle_.subscribe(boundingBoxesTopicName, 1, &ObjectTracking::boundingBoxesCallback, this);
    detectionImageSubscriber_ = nodeHandle_.subscribe(detectionImageTopicName, 1, &ObjectTracking::detectionImageCallback, this);
    multiTracker_ = cv::MultiTracker::create();


}

void ObjectTracking::cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO("[Object Tracking] camera image received");
    cv_bridge::CvImagePtr cam_image;
    try
    {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_vridge exception: %s", e.what());
        return;
    }
    if (cam_image)
    {
        camImageCopy_ = cam_image->image.clone();
    }
    return;
}
void ObjectTracking::boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes &msg)
{
    ROS_INFO("[Object Tracking] boundingBoxes received");
    boundingBoxesMsg_ = msg;
    Rect rect;
    for(int i = 0; i < boundingBoxesMsg_.bounding_boxes.size(); ++i){
        rect.x = boundingBoxesMsg_.bounding_boxes[i].xmin;
        rect.y = boundingBoxesMsg_.bounding_boxes[i].ymin;
        rect.width = boundingBoxesMsg_.bounding_boxes[i].xmax - boundingBoxesMsg_.bounding_boxes[i].xmin;
        rect.height = boundingBoxesMsg_.bounding_boxes[i].ymax - boundingBoxesMsg_.bounding_boxes[i].ymin;
        roiBoxes_.push_back(rect);
    }
    for (int i = 0; i < boundingBoxesMsg_.bounding_boxes.size(); i++)
    {
        multiTracker_->add(TrackerGOTURN::create(), detectionImageCopy_, Rect2d(roiBoxes_[i]));
    }
}
void ObjectTracking::objectCountCallback(const darknet_ros_msgs::ObjectCount &msg)
{
    ROS_INFO("[Object Tracking] object Count received");
    objectCountMsg_ = msg;
    objectCount_ = objectCountMsg_.count;
}
void ObjectTracking::detectionImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO("[Object Tracking] detection image received");
    cv_bridge::CvImagePtr detection_image;
    try
    {
        detection_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_vridge exception: %s", e.what());
        return;
    }
    if (detection_image)
    {
        detectionImageCopy_ = detection_image->image.clone();
    }
    return;
}

void *ObjectTracking::trackingInThread()
{
    multiTracker_->update(camImageCopy_);
    return 0;
}

void ObjectTracking::tracking(){
    std::thread tracking_thread;
    tracking_thread = std::thread(&ObjectTracking::trackingInThread, this);
}