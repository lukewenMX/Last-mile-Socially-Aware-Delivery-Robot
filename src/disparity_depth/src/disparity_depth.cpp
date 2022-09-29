#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <stdlib.h>
#include <stereo_msgs/DisparityImage.h>
#include <tf/transform_listener.h>

#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl_ros/impl/transforms.hpp>
#include <vector>

void disp_callback(const stereo_msgs::DisparityImageConstPtr& disp_msg) {
  cv_bridge::CvImagePtr cvimg = cv_bridge::toCvCopy(disp_msg->image);
  // std::cout << cvimg->image.type() << std::endl;
  // cv::imshow("disp",cvimg->image);
  cv::Mat depth_img(cvimg->image.rows,cvimg->image.cols,CV_16U,cv::Scalar(0));
  cv::Mat depth_show(cvimg->image.rows,cvimg->image.cols,CV_8U,cv::Scalar(0));
  for (int m = 0; m < cvimg->image.rows; ++m) {
    for (int n = 0; n < cvimg->image.cols; ++n) {
      //获得像素点对应深度
      float disp = cvimg->image.ptr<float>(m)[n];
      if(disp < disp_msg->min_disparity || disp > disp_msg->max_disparity) continue;
      float _depth = disp_msg->f * disp_msg->T / disp;
      depth_img.ptr<uint16_t>(m)[n] = (uint16_t)(_depth / 0.001);
      depth_show.ptr<uint8_t>(m)[n] = 255;
      // cloud.points.push_back(p);
      // int is_sidewalk = sidewalk_path.ptr<uint8_t>(m)[n];
      // if(is_sidewalk){
      //   // cloud_sidewalk.push_back(p);
      // }
    }
  }
  cv::imshow("depth",depth_show);
  cv::waitKey(1);
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "disparity_depth");
  ros::NodeHandle nh;
  ros::Subscriber disp_sub = nh.subscribe<stereo_msgs::DisparityImage>(
      "/camera/vga/disparity", 10, disp_callback);
  ros::spin();
  return 0;
}