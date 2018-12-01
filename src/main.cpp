/*
 * main.cpp.
 *
 * Written by: Stylianos Piperakis.
 *
 * This file creates an over-segmentation of a provided image based on the SLIC
 * superpixel algorithm, as implemented in slic.h and slic.cpp by Pascal Metter.
 */
 
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "slic.h"
#include <cstdlib>
#include <cstdio>

using namespace std;






class ImageOverSegmentation
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageOverSegmentation()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/nao_robot/camera/front/image_raw", 1, 
      &ImageOverSegmentation::imageCb, this);
    image_pub_ = it_.advertise("/nao_robot/camera/front/image_slic", 1);


  }

  ~ImageOverSegmentation()
  {

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)

    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
   

    cv::Mat img, result; 
    img=cv_ptr->image;

    int numSuperpixel=1000;
    SLIC slic;
    slic.GenerateSuperpixels(img, numSuperpixel);
	if (img.channels() == 3) 
		result = slic.GetImgWithContours(cv::Scalar(51, 51, 255));
	else
		result = slic.GetImgWithContours(cv::Scalar(128));


   
   
    cv_ptr->image=result;
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
 
   
  }
};
int main(int argc, char *argv[]) {



    ros::init(argc, argv, "slic");

    ImageOverSegmentation ioc;
    ros::spin();
    return 0;
   

    
}
