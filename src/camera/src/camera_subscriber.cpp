
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <ros/ros.h>


void callback(const sensor_msgs::ImageConstPtr& msg)
{
    static uint64_t subscribeCount=0;
    try{
        cv_bridge::CvImageConstPtr cvbImage=cv_bridge::toCvShare(msg,"bgr8");
        cv::imshow("image camera ",cvbImage->image);
        cv::waitKey(1);
        ROS_INFO("subscrib count %d-------------------\n",++subscribeCount);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cannot convert %s to bgr8\n",msg->encoding.c_str());
    }

}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"camera_subscriber");
    ros::NodeHandle camera_subscriber_node;

    image_transport::ImageTransport imageSubTrans(camera_subscriber_node);
    image_transport::Subscriber imageSub = imageSubTrans.subscribe("image_raw",10,callback);


    ros::spin();
    return 0;
}