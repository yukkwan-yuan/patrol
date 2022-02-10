#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
using namespace std;

image_transport::Subscriber sub;
ros::Publisher pub_target,pub_depth;  
ros::Subscriber sub_depth;
bool AUSCULTATION = false;
sensor_msgs::PointCloud2 depth_img;


void my_mouse_callback(int event, int x, int y, int flags, void* param){
  IplImage* image = (IplImage*) param;

  switch(event) {
    case CV_EVENT_LBUTTONDOWN:{
      std_msgs::Int16MultiArray array;
      array.data.push_back(x);
      array.data.push_back(y);
      pub_target.publish(array);
      // ROS_INFO("x: %d",x);
      // ROS_INFO("y: %d",y);
      break;
    }
    default:
      break;
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(AUSCULTATION)
  {
    AUSCULTATION=false;
    cv::Mat img;
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr=cv_bridge::toCvShare(msg, "bgr8");
    img = cv_ptr->image;
    IplImage* image = new IplImage(img);
    cvShowImage("view",image);
    while(cvGetWindowHandle("view"))
    {
      cv::setMouseCallback("view", my_mouse_callback, (void*) image);
      cvWaitKey(0);
    }
    ROS_INFO_STREAM("finish");
  }
}
void foo()
{
  char c;
  while((c = getchar()) != EOF)
  {
    if(c=='s')
    {
      pub_depth.publish(depth_img);
      AUSCULTATION = true;
    }
    if(c == 'q')
    {
      ROS_INFO("QUIT");
      break;
    }  
  }
}

void depthCB (const sensor_msgs::PointCloud2 input)
{
    depth_img = input;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  sub = it.subscribe("/camera1/color/image_raw", 1, imageCallback);
  sub_depth = nh.subscribe("/camera1/depth_registered/points", 1, depthCB);
  pub_depth = nh.advertise<sensor_msgs::PointCloud2>("ready_depth_could", 1000);
  pub_target = nh.advertise<std_msgs::Int16MultiArray>("target_xy", 1000);
  thread thread_userInput(foo);
  ros::spin();
  //cv::destroyWindow("view");
}