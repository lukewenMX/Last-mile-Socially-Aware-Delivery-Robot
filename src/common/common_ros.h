#ifndef COMMON_ROS_H
#define COMMON_ROS_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Eigen>


class CommonROS
{
public:
  /** @brief 相机内参转ros_msg
   * @param in K - [float32]相机内参
   * @param in D - [float32]相机畸变参数
   * @return sensor_msgs/CameraInfo
   */
  static sensor_msgs::CameraInfoPtr intrinsic2ros(const Eigen::Matrix3f &K, const Eigen::Matrix<float, 1, 5> &D)
  {
    assert(K.rows()==3 && K.cols()==3 && D.rows()==1 && D.cols()==5);

    sensor_msgs::CameraInfoPtr ret(new sensor_msgs::CameraInfo());
    // K: 3x3
    ret->K[0] = K(0, 0); ret->K[1] = K(0, 1); ret->K[2] = K(0, 2);
    ret->K[3] = K(1, 0); ret->K[4] = K(1, 1); ret->K[5] = K(1, 2);
    ret->K[6] = K(2, 0); ret->K[7] = K(2, 1); ret->K[8] = K(2, 2);

    // D: 1x5
    for (int c = 0; c < D.cols(); c++) {
      ret->D.push_back(D(0, c));
    }
    return ret;
  }
};

#endif // COMMON_ROS_H
