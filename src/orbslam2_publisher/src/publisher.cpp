#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

//http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
//也真是神奇到底为什么会这样呢

#include <opencv2/opencv.hpp>
#include<fstream>


void loadImage(const std::string &strImagePath, const std::string &strTimePath,
               std::vector<std::string>& vstrImages,std::vector<double>& vTimesStamp);


int main(int argc, char** argv)
{
  // Check if video source has been passed as a parameter
  if(argv[1] == NULL) 
  {
      std::cout <<"-----------------请输入参数-------------------"<<std::endl;
      return 1;
  }

//节点名称
  ros::init(argc, argv, "orbslam2_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);        //话题名称
  image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);

  //load image
  std::vector<std::string> vstrImage;
  std::vector<double> vTimesStamp;
  loadImage(argv[1],argv[2],vstrImage,vTimesStamp);
  printf("line %d  images %ld timeStamp %ld\n",__LINE__, vstrImage.size(),vTimesStamp.size());

//   // Convert the passed as command line parameter index for the video device to an integer
//   std::istringstream video_sourceCmd(argv[1]);
//   int video_source;
//   // Check if it is indeed a number
//   //直接将字符串转化为整形数据
//   if(!(video_sourceCmd >> video_source)) return 1;

//   cv::VideoCapture cap(video_source);
//   // Check if video device can be opened with the given index
//   if(!cap.isOpened()) return 1;


  cv::Mat frame;
  std::vector<std::string>::iterator ite=vstrImage.begin();

  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(20);
  while (nh.ok()) {
    // cap >> frame;
    frame = cv::imread(*ite);
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::imshow("camera image",frame);
      cv::waitKey(1);
    }
    if(++ite==vstrImage.end())
        break;

    ros::spinOnce();
    loop_rate.sleep();
  }
}

/**
 * 
 * <sstream> 定义了三个类：istringstream、ostringstream 和 stringstream，分别用来进行流的输入、
 * 输出和输入输出操作。本文以 stringstream 为主，介绍流的输入和输出操作。

    <sstream> 主要用来进行数据类型转换，由于 <sstream> 使用 string 对象来代替字符数组（snprintf方式），
    就避免缓冲区溢出的危险；而且，因为传入参数和目标对象的类型会被自动推导出来，所以不存在错误的格式化符的问题。
    简单说，相比c库的数据类型转换而言，<sstream> 更加安全、自动和直接。
    原文链接：https://blog.csdn.net/liitdar/article/details/82598039
 * 
 * 
*/

void loadImage(const std::string &strImagePath, const std::string &strTimePath,
               std::vector<std::string>& vstrImages,std::vector<double>& vTimesStamp)
{
    std::ifstream fTimes;
    fTimes.open(strTimePath.c_str());
    vTimesStamp.reserve(5000);
    vstrImages.reserve(5000);

    while(!fTimes.eof())
    {
        std::string s;
        std::getline(fTimes,s);
        // printf("%d %s\n",__LINE__,s.c_str());
        if(!s.empty())
        {
            std::stringstream ss;
            ss<<s;
            vstrImages.push_back(strImagePath+"/"+ss.str()+".png");
            double t;
            ss>>t;
            vTimesStamp.push_back(t);
        }

    }
}