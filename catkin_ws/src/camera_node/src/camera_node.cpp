#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;
int main(int argc, char **argv)
{
  printf("start run\n");
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/rgb/image_raw", 1);
  int count  = 0;
  // play a video
  VideoCapture cap("/home/lilei/Downloads/dog.mp4");
  if (!cap.isOpened())
  {
    cout << "can not open video file" << endl;
  }
  while (1)
  {
    Mat frame;
    cap >> frame;
    if (frame.empty())
      break;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    ros::Rate loop_rate(5);
    while (nh.ok())
    {
      pub.publish(msg);
      ROS_INFO("frame %d has been published successfully!\n", ++count);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}
