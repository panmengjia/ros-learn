#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_publisher");

  int video_device;
  int frame_rate;
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image_raw", 10);

  nh.param<int>("video_device", video_device, 0);
  nh.param<int>("frame_rate", frame_rate, 30);

  cv::VideoCapture TheVideoCapturer;
  TheVideoCapturer.open(video_device);
  if (!TheVideoCapturer.isOpened()) {
    ROS_WARN("Open camera device%d error!", video_device);
    return false;
  }
  ROS_INFO("Camera device%d openned, fps=%d", video_device, frame_rate);

//   ros::Rate loop_rate(frame_rate); 
  ros::Rate loop_rate(10);
  uint32_t publishCount=0;
  while (nh.ok()) {
    if (TheVideoCapturer.grab()) {
      cv::Mat image;
      TheVideoCapturer.retrieve(image);
      cv_bridge::CvImage out_msg;
      out_msg.header.stamp = ros::Time::now();
      out_msg.encoding = sensor_msgs::image_encodings::BGR8;
      out_msg.image = image;
      pub.publish(out_msg.toImageMsg());
      ROS_INFO("publish count %d-------------------\n",++publishCount);
      cv::imshow("publish camera",image);
      cv::waitKey(1);
    }
    loop_rate.sleep();
  }
  return 0;
}