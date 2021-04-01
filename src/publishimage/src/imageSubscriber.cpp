#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
static const char WINDOW[] = "Image window";
static void help()
{
    printf("\nThis program demonstrates converting OpenCV Image to ROS Image messages  \n"
        );
 
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  static uint32_t count=0;
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    ROS_INFO("--------------count   %d------------------\n",++count);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
 

 int main(int argc, char** argv)
 {

    help();
    ros::init(argc, argv, "image_subscriber");

    ros::NodeHandle node_subscriber;

    image_transport::ImageTransport it(node_subscriber);
    image_transport::Subscriber sub = it.subscribe("OutImage", 1, imageCallback);


    ros::spin();
    cv::destroyWindow("view");
    return 0;
 }