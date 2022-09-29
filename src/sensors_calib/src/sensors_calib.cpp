/** @brief 相机-雷达套件标定
 *
 *  给定参数文件标定相机-雷达，重新发布数据
 *  输出的图像数据都是通过相机参数重新较正
 *  输出的深度图都是由雷达坐标系变换得到: pointsUV = K * T * pointsL
 *
 * 数据处理流程
 * ==========
 *  intrinsic_rgb ----------> K,D -o-\
 *  intrinsic_thermal ------> K,D -|-|-\
 *  extrinsic_livox2rgb ----> T ---|-|-|-\
 *  homograph_thermal2rgb --> H ---|-|-o |
 *                                 | | | |
 *  RGB[sensor_msg/Image] ---------o-|-|-|--- RGBProcess -----o->Publish RGB[sensor_msg/Image, RGB]
 *                                   | | |                    \->Publish RGBCameraInfo[sensor_msg/CameraInfo]
 *  sensor_msg/Image[Thermal] -------|-o-|--- ThermalProcess -o->Publish Thermal[sensor_msg/Image, RGB]
 *                                   |   |                    \->Publish ThermalCameraInfo[sensor_msg/CameraInfo]
 *  sensor_msg/PointCloud2 ----------o---o--- cloud2depth ----o->Publish Depth[snesor_msg/Image, float32]
 *                                       |                    \->Publish DepthCameraInfo[snesor_msg/CameraInfo]
 *                                       \--- cloud2intensity -->Publish Intensity[snesor_msg/Image, float32]
 * RGBProcess
 * ----------
 *  Input : K[RGB], D[RGB]
 *          image [sensor_msg/Image]
 *  Output: image_rect[sensor_msg/Image]
 *          camera_info_rect[sensor_msg/CameraInfo]
 *  Process:
 *   image_rect <- undistort(image, K, D)       // 去畸变
 *   camera_info_rect <- FillCameraInfo(K, D)   // 填充CameraInfo结构体
 *
 * ThermalProcess
 * --------------
 *  Input : K[Thermal], D[Thermal], H[Thermal2RGB]
 *          image [sensor_msg/Image]
 *          rgb_size[cv::Size]
 *  Output: image_rect[sensor_msg/Image]
 *          camera_info_rect[sensor_msg/CameraInfo]
 *  Process:
 *    undistort_image <- undistort(image, K, D)                   // 去畸变
 *    image_rect <- warpPrrpective(undistort_image, H, rgb_size)  // 单应性矩阵变换
 *    camera_info_rect <- FillCameraInfo(K, D)                    // 填充CameraInfo结构体
 *
 * cloud2depth
 * -----------
 *  Input : K[RGB]
 *          T[Matrix4f]     // extrinsic_livox2rgb
 *          cloud[sensor_msg/PointCloud2]
 *  Output: depth[sensor_msg/Image, float32]
 *          depth_camera_info[sensor_msg/CameraInfo]
 *          intensity[sensor_msg/Image, float32]
 *  Process:
 *    cvPointsL, cvIntensity <- Extract(cloud)  # cvPointsW: 4xN, 提取点云齐次坐标； cvIntensify: 1xN, 对应强度值
 *    cvPointsC <- T * cvPointsL                # T: 4x4, cvPointsC: 4xN, 雷达坐标转换为相机坐标
 *    cvPointsUV <- [K, 0] * cvPointsC # cvPointsUV: 3xN, [K, 0]: 3x4, 相机坐标转换为像素坐标
 *    // 生成深度图
 *    for i < N:
 *      u = cvPointsUV[0, i] / cvPointsUV[2, i]  // u 坐标
 *      v = cvPointsUV[1, i] / cvPointsUV[2, i]  // v 坐标
 *      // 边界检测
 *      if u>=0 && u<rgb_size.width && v>=0 && v<rgb_size.height:
 *        depth[v, u] = cvPointsUV[2, i]
 *        intensify[v, u] = cvIntensify[i]
 *    depth_camera_info = FillCameraInfo(K)   // 填充CameraInfo结构体
 *
 *
 * depth2cloud
 * -----------
 *  Input  : depth[sensor_msg/Image]
 *           depth_camera_info[sensor_msgs/Image]
 *           T[Matrix4f]     // extrinsic_livox2rgb
 *  Output : PointsL[Livox coordinate system]
 *  Process:
 *    K <- depth_camera_info
 *    cvPointsUV <- Extract(depth)
 *    cvPointsUV[0, i] = cvPointsUV[0, i] * cvPointsUV[2, i]
 *    cvPoitnsUV[1, i] = cvPointsUV[1, i] * cvPointsUV[2, i]
 *    PointsC = K_inv * cvPointsUV
 *    PointsL = T_inv * PointsC
 *
 * Parameters:
 * ===========
 *  frame_id                  - ROS Message header.frame_id
 *  verbose                   - 日志输出等级
 *  # output_depth_on_rgb, output_depth_on_thermal, rgb_fuse_with_thermal 控制输出模式
 *  output_depth_on_rgb       - 是否输出深度图(基于彩色相机参数)
 *  output_depth_on_thermal   - 是否输出深度图(基于热感相机参数)
 *  rgb_fuse_with_thermal     - 是否把热感相机融入到彩色图中: RGB = RGB+Thermal
 *  intrinsic_rgb             - 彩色相机内参文件, default: <sensors_calib>/config/intrinsic_rgb.txt
 *  intrinsic_thermal         - 热感相机内参文件, default: <sensors_calib>/config/intrinsic_thermal.txt
 *  extinsic_livox2rgb        - 雷达-相机外参文件, default: <sensors_calib>/config/extrinsic_livox2rgb.txt
 *  extrinsic_thermal2rgb     - 热感-彩色相机外参文件, <snesors_calib>/config/extrinsic_thermal2rgb.txt
 *  homograph_thermal2rgb     - 热感-彩色相机单应性矩阵文件, <snesors_calib>/config/homograph_thermal2rgb.txt
 *  # HSV 变换： 可以构造夜晚效果
 *  # ------------------------
 *  #   RGB => HSV
 *  #   HSV[:, :, 0] = ((int)(HSV[:, :, 0] * HSV_h1)) * HSV_h2
 *  #   HSV[:, :, 1] = ((int)(HSV[:, :, 1] * HSV_s1)) * HSV_s2
 *  #   HSV[:, :, 2] = ((int)(HSV[:, :, 2] * HSV_v1)) * HSV_v2
 *  HSV_h1                    - RGB HSV变换: H 系数: [0-1]
 *  HSV_h2                    - RGB HSV变换: H 系数: >=0 放大倍数
 *  HSV_s1                    - RGB HSV变换: S 系数: [0-1]
 *  HSV_s2                    - RGB HSV变换: S 系数: >=0 放大倍数
 *  HSV_v1                    - RGB HSV变换: V 系数: [0-1]
 *  HSV_v2                    - RGB HSV变换: V 系数: >=0 放大倍数
 *  rgb_width                 - 彩色图像宽度， default: 640
 *  rgb_height                - 彩色图像高度, default: 480
 *  thermal_width             - 热感图像宽度, default: 640
 *  thermal_height            - 热感图像高度，default: 512
 *  min_distance              - 最小测距(livox), default: 1.0m
 *  max_distance              - 最大测距(livox), default: 300.0m
 *  tp_in_rgb                 - 彩色相机订阅主题, default: /rgb_cam/image_raw
 *  tp_in_rgb_camerainfo      - 彩色相机参数订阅主题, default: /rgb_cam/camerainfo
 *  tp_in_thermal             - 热感相机订阅主题, default: /thermal_cam/thermal_image
 *  tp_in_thermal_camerainfo  - 热感相机参数订阅主题, default: /thermal_cam/camerainfo
 *  tp_in_livox               - 雷达点云数据订阅主题, default: /livox/lidar
 *  tp_out_rgb                - 彩色相机重新发布主题, default: /rgb_cam/image_raw
 *  tp_out_rgb_camerainfo     - 彩色相机参数重新发布主题, default: /rgb_cam/camerainfo
 *  tp_out_thermal            - 热感相机重新发布主题， default: /thermal_cam/thermal_image
 *  tp_out_thermal_camerainfo - 热感相机参数重新发布主题, default: /thermal_cam/camerainfo
 *  tp_out_depth_rgb          - 深度图发布主题(基于彩色相机参数), default: /livox/depth_rgb
 *  tp_out_depth_thermal      - 深度图发布主题(基于热感相机参数), default: /livox/depth_thermal
 *  tp_out_intensity_rgb      - 强度值图发布主题(基于彩色相机参数): default: /livox/intensity_rgb
 *  tp_out_intensity_thermal  - 强度值图发布主题(基于热感相机参数): default: /livox/intensity_thermal
 *
 *  说明: 相机图像主题以 /compressed 结束时为压缩格式
 *
 * Subscribe
 * ==========
 *  彩色相机图像[sensors_msg/Image | sensor_msgs/CompressedImage]
 *    /rgb_cam/image_raw or /rgb_cam/image_raw/compressed
 *  彩色相机参数[sensor_msgs/CameraInfo]
 *    /rgb_cam/camerainfo
 *  热感相机图像[sensors_msg/Image | sensor_msgs/CompressedImage]
 *    /thermal_cam/thermal_image or /thermal_cam/thermal_image/compressed
 *  热感相机参数[sensor_msgs/CameraInfo]
 *    /thermal_cam/camerainfo
 *  雷达点云数据[sensor_msgs/PointCloud2]
 *    /livox/lidar
 *
 *  说明:真实主题路径以实际设置为准
 *
 * Publish
 * ========
 *  彩色相机图像[sensors_msg/Image | sensor_msgs/CompressedImage]
 *    /rgb_cam/image_raw or /rgb_cam/image_raw/compressed
 *  彩色相机参数[sensor_msgs/CameraInfo]
 *    /rgb_cam/camerainfo
 *  热感相机图像[sensors_msg/Image | sensor_msgs/CompressedImage]
 *    /thermal_cam/thermal_image or /thermal_cam/thermal_image/compressed
 *  热感相机参数[sensor_msgs/CameraInfo]
 *    /thermal_cam/camerainfo
 *  雷达点云数据[sensor_msgs/PointCloud2]
 *    /livox/lidar
 *  深度图(基于彩色相机参数)[sensor_msgs/Image, float32]
 *    /livox/depth_rgb
 *  深度图(基于热感相机参数)[sensor_msgs/Image, float32]
 *    /livox/depth_thermal
 *  强度值图(基于彩色相机参数)[sensor_msgs/Image, float32]
 *    /livox/intensity_rgb
 *  强度值图(基于热感相机参数)[sensor_msgs/Image, float32]
 *    /livox/intensity_thermal
 *
 *  说明:真实主题路径以实际设置为准
 *
 * Verbose[debug]
 * ==============
 *   verbose=2
 *
 * 更新记录
 * =======
 *  - 添加强度值输出
 */

#include "../common/common.h"
#include "../common/common_ros.h"

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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

/** 参数配置
 */
std::string frame_id_;
int verbose_ = 0;
//int mode_ = 0;                          // 运行模式
bool output_depth_on_rgb_ = true;       // 输出深度图(基于彩色相机参数)
bool output_depth_on_thermal_ = true;   // 输出深度图(基于热感相机参数)
bool rgb_fuse_with_thermal_ = true;     // 热感相机融入到彩色图中: RGB = RGB+Thermal
std::string intrinsic_rgb_;             // 彩色相机内参文件
std::string intrinsic_thermal_;         // 热感相机内参文件
std::string extrinsic_livox2rgb_;       // 雷达-彩色相机外参文件
std::string extrinsic_livox2thermal_;   // 雷达-热感相机外参文件
std::string extrinsic_thermal2rgb_;     // 热感-彩色相机外参文件
std::string homograph_thermal2rgb_;     // 热感-彩色相机单应性矩阵文件
float HSV_h1_ = 1.0 ;                   // RGB HSV变换: H 系数: [0-1]
int HSV_h2_ = 1 ;                       // RGB HSV变换: H 系数: >=0 放大倍数
float HSV_s1_ = 1.0 ;                   // RGB HSV变换: S 系数: [0-1]
int HSV_s2_ = 1 ;                       // RGB HSV变换: S 系数: >=0 放大倍数
float HSV_v1_ = 1.0 ;                   // RGB HSV变换: V 系数: [0-1]
int HSV_v2_ = 1 ;                       // RGB HSV变换: V 系数: >=0 放大倍数

int rgb_width_ = 640;
int rgb_height_ = 480 ;
int thermal_width_ = 640;
int thermal_height_ = 512;
float min_distance_ = 1.0;
float max_distance_ = 300.0;
std::string tp_in_rgb_;                 // 订阅主题: 彩色相机
std::string tp_in_rgb_camerainfo_;      // 订阅主题: 彩色相机内参
std::string tp_in_thermal_;             // 订阅主题: 热感相机
std::string tp_in_thermal_camerainfo_;  // 订阅主题: 热感相机内参
std::string tp_in_livox_;               // 订阅主题: 雷达点云
std::string tp_out_rgb_;                // 发布主题: 彩色相机
std::string tp_out_rgb_camerainfo_;     // 发布主题: 彩色相机内参
std::string tp_out_thermal_;            // 发布主题: 热感相机
std::string tp_out_thermal_camerainfo_; // 发布主题: 热感相机参数
std::string tp_out_depth_rgb_;          // 发布主题: 深度图(基于彩色相机参数)
std::string tp_out_depth_thermal_;      // 发布主题: 深度图(基于热感相机参数)
std::string tp_out_intensity_rgb_;      // 发布主题: 强度图(基于彩色相机参数)
std::string tp_out_intensity_thermal_;  // 发布主题: 强度图(基于热感相机参数)

/** 订阅/发布主题
 */
ros::Subscriber sub_rgb_;                 // 订阅:彩色图像
ros::Subscriber sub_rgb_camerainfo_;      // 订阅:彩色相机参数
ros::Subscriber sub_thermal_;             // 订阅:热感图像
ros::Subscriber sub_thermal_camerainfo_;  // 订阅:热感相机参数
ros::Subscriber sub_livox_;               // 订阅:雷达点云
ros::Publisher  pub_rgb_;                 // 发布:彩色图像
ros::Publisher  pub_rgb_camerainfo_;      // 发布:彩色相机参数
ros::Publisher  pub_thermal_;             // 发布:热感图像
ros::Publisher  pub_thermal_camerainfo_;  // 发布:热感相机参数
ros::Publisher  pub_depth_rgb_;           // 发布:深度图像(基于彩色相机参数)
ros::Publisher  pub_depth_thermal_;       // 发布:深度图像(基于热感相机参数)
ros::Publisher  pub_intensity_rgb_;       // 发布:强度图像(基于彩色相机参数)
ros::Publisher  pub_intensity_thermal_;   // 发布:强度图像(基于热感相机参数)

/** 相机-雷达内外参他
 */
Eigen::Matrix3f             K_RGB_;               // 彩色相机内参
Eigen::Matrix3f             K_RGB_inv_;           // 彩色相机内参逆
Eigen::Matrix<float, 3, 4>  Ka_RGB_;              // 彩色相机内参:增广矩阵， 最右列为0
Eigen::Matrix<float, 1, 5>  D_RGB_;               // 彩色相机畸变参数

Eigen::Matrix3f             K_Thermal_;           // 热感相机内参
Eigen::Matrix3f             K_Thermal_inv_;       // 热感相机内参逆
Eigen::Matrix<float, 3, 4>  Ka_Thermal_;          // 热感相机内参:增广矩阵， 最右列为0
Eigen::Matrix<float, 1, 5>  D_Thermal_;           // 热感相机畸变参数

Eigen::Matrix4f             T_Thermal2RGB_;       // 热感-彩色相机外参
Eigen::Matrix4f             T_Thermal2RGB_inv_;   // 热感-彩色相机外参逆

Eigen::Matrix4f             T_Livox2RGB_;         // 雷达-彩色相机外参
Eigen::Matrix4f             T_Livox2RGB_inv_;     // 雷达-彩色相机外参逆

Eigen::Matrix4f             T_Livox2Thermal_;     // 雷达-热感相机外参
Eigen::Matrix4f             T_Livox2Thermal_inv_; // 雷达-热感相机外参逆

Eigen::Matrix3f             H_Thermal2RGB_;       // 热感-彩色相机单应性矩阵
Eigen::Matrix3f             H_Thermal2RGB_inv_;   // 热感-彩色相机单应性矩阵逆

cv::Mat  cvK_RGB_;              // 彩色相机内参: 3x3
cv::Mat  cvD_RGB_;              // 彩色相机畸变参数: 1x5
cv::Mat  cvK_Thermal_;          // 热感相机内参: 3x3
cv::Mat  cvD_Thermal_;          // 热感相机畸变参数: 1x5

cv::Mat  cvT_Thermal2RGB_;      // 热感-彩色相机外参: 4x4
cv::Mat  cvT_Livox2RGB_;        // 雷达-彩色相机外参:4x4
cv::Mat  cvT_Livox2Thermal_;    // 雷达-热感相机外参:4x4

cv::Mat  cvH_Thermal2RGB_;      // 热感-彩色相机单应性矩阵:3x3

cv::Size rgb_size_(rgb_width_, rgb_height_);              // RGB图像尺寸
cv::Size thermal_size_(thermal_width_, thermal_height_);  // Thermal图像尺寸

// 帧序号
long rgb_frame_index_ = 0;
long thermal_frame_index_ = 0;
long livox_frame_index_ = 0;

/** @brief 彩色相机较正
 */
cv::Mat rgb_calib(const cv::Mat &cv_image)
{
  // HSV 变换
  if (HSV_h1_ != 1.0 || HSV_s1_ != 1.0 || HSV_v1_ != 1.0) {
    cv::Mat hsv;
    cv::cvtColor(cv_image, hsv, CV_RGB2HSV);
    int rows = cv_image.rows;
    int cols = cv_image.cols;
    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < cols; c++) {
        hsv.at<cv::Vec3b>(r, c)[0] = ((int)(hsv.at<cv::Vec3b>(r, c)[0] * HSV_h1_)) * HSV_h2_;
        hsv.at<cv::Vec3b>(r, c)[1] = ((int)(hsv.at<cv::Vec3b>(r, c)[1] * HSV_s1_)) * HSV_s2_;
        hsv.at<cv::Vec3b>(r, c)[2] = ((int)(hsv.at<cv::Vec3b>(r, c)[2] * HSV_v1_)) * HSV_v2_;
      }
    }
    cv::cvtColor(hsv, cv_image, CV_HSV2RGB);
  }
  // 去畸变
  cv::Mat undistort_image;
  cv::undistort(cv_image, undistort_image, cvK_RGB_, cvD_RGB_);

  return undistort_image;
}

/** @brief 热感相机较正
 */
cv::Mat thermal_calib(const cv::Mat &cv_image)
{
  // Image 去畸变
  cv::Mat undistort_image;
  cv::undistort(cv_image, undistort_image, cvK_RGB_, cvD_RGB_);
  // 标定 Thermal2RGB
  cv::Mat image;
  cv::warpPerspective(undistort_image, image, cvH_Thermal2RGB_, rgb_size_);
  return image;
}


/** @brief 彩色相机回调函数
 * 原始图去畸变和内参修正 后 重新发布
 */
void rgb_callback(const sensor_msgs::ImageConstPtr& msg)
{
  if (verbose_>0) {
    ROS_INFO("rgb_callback frame_index: %ld, stamp:(%d, %d)",
             rgb_frame_index_++, msg->header.stamp.sec, msg->header.stamp.nsec);
  }

  // image_msg => OpenCV
  cv::Mat cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
  cv::Mat undistort_image = rgb_calib(cv_image);
  // OpenCV => image_msg
  sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(msg->header, "bgr8", undistort_image).toImageMsg();
  // 填充 CameraInfo: (K, D) => sensors_msg/CameraInfo
  sensor_msgs::CameraInfoPtr ros_camerainfo = CommonROS::intrinsic2ros(K_RGB_, D_RGB_);
  ros_camerainfo->width = rgb_width_;
  ros_camerainfo->height = rgb_height_;

  if (frame_id_ !="" ) {
    ros_image->header.frame_id = frame_id_;
    ros_camerainfo->header.frame_id = frame_id_;
  }

  // 重新发布 Image
  pub_rgb_.publish(ros_image);
  // 重新发布 CameraInfo
  pub_rgb_camerainfo_.publish(ros_camerainfo);
}


/** @brief 彩色相机融合热感相机回调函数
 * 原始图去畸变和内参修正 后 重新发布
 */
void rgb_fuse_with_thermal_callback(const sensor_msgs::ImageConstPtr& msg_rgb,
                                    const sensor_msgs::ImageConstPtr& msg_thermal)
{
  if (verbose_>0) {
    ROS_INFO("rgb_fuse_with_thermal_callback frame_index: %ld, stamp:(%d, %d)",
             rgb_frame_index_++, msg_rgb->header.stamp.sec, msg_rgb->header.stamp.nsec);
  }

  // image_msg => OpenCV
  cv::Mat cv_image_rgb = rgb_calib(cv_bridge::toCvShare(msg_rgb, "bgr8")->image);
  cv::Mat cv_image_thermal = thermal_calib(cv_bridge::toCvShare(msg_thermal, "bgr8")->image);

  // 融合
  cv_image_rgb.convertTo(cv_image_rgb, CV_32FC3);
  cv_image_thermal.convertTo(cv_image_thermal, CV_32FC3);
  cv::addWeighted(cv_image_rgb, 0.5, cv_image_thermal, 0.5, 0, cv_image_rgb, CV_32FC3);
  cv::normalize(cv_image_rgb, cv_image_rgb, 0, 255, cv::NORM_MINMAX, CV_8UC3);

  // OpenCV => image_msg
  sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(msg_rgb->header, "bgr8", cv_image_rgb).toImageMsg();
  // 填充 CameraInfo: (K, D) => sensors_msg/CameraInfo
  sensor_msgs::CameraInfoPtr ros_camerainfo = CommonROS::intrinsic2ros(K_RGB_, D_RGB_);
  ros_camerainfo->width = rgb_width_;
  ros_camerainfo->height = rgb_height_;

  if (frame_id_ !="" ) {
    ros_image->header.frame_id = frame_id_;
    ros_camerainfo->header.frame_id = frame_id_;
  }

  // 重新发布 Image
  pub_rgb_.publish(ros_image);
  // 重新发布 CameraInfo
  pub_rgb_camerainfo_.publish(ros_camerainfo);
}


/** @brief 热感相机回调函数
 *  原始图去畸变内参修正+与彩色相机的外参修正 后 重新发布
 */
void thermal_callback(const sensor_msgs::ImageConstPtr& msg)
{
  if (verbose_>0) {
    ROS_INFO("thermal_callback frame_index: %ld, stamp:(%d, %d)",
             thermal_frame_index_++, msg->header.stamp.sec, msg->header.stamp.nsec);
  }

  // image_msg => OpenCV
  cv::Mat cv_image = thermal_calib(cv_bridge::toCvShare(msg, "bgr8")->image);
  // OpenCV => image_msg
  sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(msg->header, "bgr8", cv_image).toImageMsg();
  // 填充 CameraInfo: (K, D) => sensors_msg/CameraInfo
  sensor_msgs::CameraInfoPtr ros_camerainfo = CommonROS::intrinsic2ros(K_Thermal_, D_Thermal_);
  ros_camerainfo->width = cv_image.cols;  //thermal_width_;
  ros_camerainfo->height = cv_image.rows; //thermal_height_;

  if (frame_id_ !="" ) {
    ros_image->header.frame_id = frame_id_;
    ros_camerainfo->header.frame_id = frame_id_;
  }

  // 重新发布 Image
  pub_thermal_.publish(ros_image);
  // 重新发布 CameraInfo
  pub_thermal_camerainfo_.publish(ros_camerainfo);
}

/** @brief 点云投影深度图
 *  1。点云投影到深度图
 *  2。使用相机参数作为深度图相机参数
 *  3。重新发布主题: 深度图，强度值图，深度图相机参数
 */
void livox2depth(const sensor_msgs::PointCloud2::ConstPtr& msg,
                 const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in,
                 const cv::Size & cam_size,
                 const Eigen::Matrix3f &K, const Eigen::Matrix<float, 3, 4> &Ka, const Eigen::Matrix3f &K_inv,
                 const Eigen::Matrix4f &T_l2c, const Eigen::Matrix4f &T_l2c_inv,
                 ros::Publisher &pub_depth,
                 ros::Publisher &pub_intensity)
{
  // 生成深度图, 强度值图
  cv::Mat depth = cv::Mat::zeros(cam_size.height, cam_size.width, CV_32FC1);
  cv::Mat intensity = cv::Mat::zeros(cam_size.height, cam_size.width, CV_32FC1);
  CommonFuns::cloud2depth(cloud_in, depth, intensity, K, Ka, T_l2c, min_distance_, max_distance_);

  // 转换ROS格式
  sensor_msgs::ImagePtr ros_depth = cv_bridge::CvImage(msg->header, "32FC1", depth).toImageMsg();
  sensor_msgs::ImagePtr ros_intensity = cv_bridge::CvImage(msg->header, "32FC1", intensity).toImageMsg();

  if (frame_id_ !="" ) {
    ros_depth->header.frame_id = frame_id_;
    ros_intensity->header.frame_id = frame_id_;
  }

  // 发布深度图
  pub_depth.publish(ros_depth);
  // 发布强度值图
  pub_intensity.publish(ros_intensity);
}


/** @brief 雷达点云回调函数
 *  1。点云投影到深度图
 *  2。使用彩色相机参数作为深度图相机参数
 *  3。重新发布主题: 深度图，强度值图，深度图相机参数
 */
void livox_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if (verbose_>0) {
    ROS_INFO("livox_callback frame_index: %ld, stamp:(%d, %d)",
             livox_frame_index_++, msg->header.stamp.sec, msg->header.stamp.nsec);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_in);

  if (output_depth_on_rgb_ || rgb_fuse_with_thermal_) {
    // 生成深度图/强度值图(基于彩色相机参数)
    livox2depth(msg, cloud_in, rgb_size_,
                     K_RGB_, Ka_RGB_, K_RGB_inv_,
                     T_Livox2RGB_ , T_Livox2RGB_inv_,
                     pub_depth_rgb_, pub_intensity_rgb_);
  }
  if (output_depth_on_thermal_) {
    // 生成深度图/强度值图(基于热感相机参数)
    livox2depth(msg, cloud_in, thermal_size_,
                     K_Thermal_, Ka_Thermal_, K_Thermal_inv_,
                     T_Livox2Thermal_ , T_Livox2Thermal_inv_,
                     pub_depth_thermal_, pub_intensity_thermal_);
  }
}


/** @brief 主函数
 */
int main(int argc, char **argv)
{
  // ROS节点初始化
  // ===========
  ros::init(argc, argv, "sensors_calib");
  ROS_INFO("sensors_calib initializtion!");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // 读取ROS参数
  // ==========
  if (ros::param::get("~frame_id", frame_id_)) {
    ROS_INFO("Retrived param 'frame_id': %s", frame_id_.c_str());
  }
  if (ros::param::get("~verbose", verbose_)) {
    ROS_INFO("Retrived param 'verbose': %d", verbose_);
  }
  if (ros::param::get("~output_depth_on_rgb", output_depth_on_rgb_)) {
    ROS_INFO("Retrived param 'output_depth_on_rgb': %d", output_depth_on_rgb_);
  }
  if (ros::param::get("~output_depth_on_thermal", output_depth_on_thermal_)) {
    ROS_INFO("Retrived param 'output_depth_on_thermal': %d", output_depth_on_thermal_);
  }
  if (ros::param::get("~rgb_fuse_with_thermal", rgb_fuse_with_thermal_)) {
    ROS_INFO("Retrived param 'rgb_fuse_with_thermal': %d", rgb_fuse_with_thermal_);
  }
  if (ros::param::get("~intrinsic_rgb", intrinsic_rgb_)) {
    ROS_INFO("Retrived param 'intrinsic_rgb': %s", intrinsic_rgb_.c_str());
  }
  if (ros::param::get("~intrinsic_thermal", intrinsic_thermal_)) {
    ROS_INFO("Retrived param 'intrinsic_thermal': %s", intrinsic_thermal_.c_str());
  }
  if (ros::param::get("~extrinsic_livox2rgb", extrinsic_livox2rgb_)) {
    ROS_INFO("Retrived param 'extrinsic_livox2rgb': %s", extrinsic_livox2rgb_.c_str());
  }
  if (ros::param::get("~extrinsic_livox2thermal", extrinsic_livox2thermal_)) {
    ROS_INFO("Retrived param 'extrinsic_livox2thermal': %s", extrinsic_livox2thermal_.c_str());
  }
  if (ros::param::get("~extrinsic_thermal2rgb", extrinsic_thermal2rgb_)) {
    ROS_INFO("Retrived param 'extrinsic_thermal2rgb': %s", extrinsic_thermal2rgb_.c_str());
  }
  if (ros::param::get("~homograph_thermal2rgb", homograph_thermal2rgb_)) {
    ROS_INFO("Retrived param 'homograph_thermal2rgb': %s", homograph_thermal2rgb_.c_str());
  }
  if (ros::param::get("~HSV_h1", HSV_h1_)) {
    ROS_INFO("Retrived param 'HSV_h1': %f", HSV_h1_);
  }
  if (ros::param::get("~HSV_h2", HSV_h2_)) {
    ROS_INFO("Retrived param 'HSV_h2': %d", HSV_h2_);
  }
  if (ros::param::get("~HSV_s1", HSV_s1_)) {
    ROS_INFO("Retrived param 'HSV_s1': %f", HSV_s1_);
  }
  if (ros::param::get("~HSV_s2", HSV_s2_)) {
    ROS_INFO("Retrived param 'HSV_s2': %d", HSV_s2_);
  }
  if (ros::param::get("~HSV_v1", HSV_v1_)) {
    ROS_INFO("Retrived param 'HSV_v1': %f", HSV_v1_);
  }
  if (ros::param::get("~HSV_v2", HSV_v2_)) {
    ROS_INFO("Retrived param 'HSV_v2': %d", HSV_v2_);
  }
  if (ros::param::get("~rgb_width_", rgb_width_)) {
    ROS_INFO("Retrived param 'rgb_width': %d", rgb_width_);
  }
  if (ros::param::get("~rgb_height_", rgb_height_)) {
    ROS_INFO("Retrived param 'rgb_height': %d", rgb_height_);
  }
  if (ros::param::get("~thermal_width_", thermal_width_)) {
    ROS_INFO("Retrived param 'thermal_width': %d", thermal_width_);
  }
  if (ros::param::get("~thermal_height_", thermal_height_)) {
    ROS_INFO("Retrived param 'thermal_height': %d", thermal_height_);
  }
  if (ros::param::get("~min_distance_", min_distance_)) {
    ROS_INFO("Retrived param 'min_distance': %f", min_distance_);
  }
  if (ros::param::get("~max_distance_", max_distance_)) {
    ROS_INFO("Retrived param 'max_distance': %f", max_distance_);
  }
  if (ros::param::get("~tp_in_rgb", tp_in_rgb_)) {
    ROS_INFO("Retrived param 'tp_in_rgb': %s", tp_in_rgb_.c_str());
  }
  if (ros::param::get("~tp_in_rgb_camerainfo", tp_in_rgb_camerainfo_)) {
    ROS_INFO("Retrived param 'tp_in_rgb_camerainfo': %s", tp_in_rgb_camerainfo_.c_str());
  }
  if (ros::param::get("~tp_in_thermal", tp_in_thermal_)) {
    ROS_INFO("Retrived param 'tp_in_thermal': %s", tp_in_thermal_.c_str());
  }
  if (ros::param::get("~tp_in_thermal_camerainfo", tp_in_thermal_camerainfo_)) {
    ROS_INFO("Retrived param 'tp_in_thermal_camerainfo': %s", tp_in_thermal_camerainfo_.c_str());
  }
  if (ros::param::get("~tp_in_livox", tp_in_livox_)) {
    ROS_INFO("Retrived param 'tp_in_livox': %s", tp_in_livox_.c_str());
  }
  if (ros::param::get("~tp_out_rgb", tp_out_rgb_)) {
    ROS_INFO("Retrived param 'tp_out_rgb': %s", tp_out_rgb_.c_str());
  }
  if (ros::param::get("~tp_out_rgb_camerainfo", tp_out_rgb_camerainfo_)) {
    ROS_INFO("Retrived param 'tp_out_rgb_camerainfo': %s", tp_out_rgb_camerainfo_.c_str());
  }
  if (ros::param::get("~tp_out_thermal", tp_out_thermal_)) {
    ROS_INFO("Retrived param 'tp_out_thermal': %s", tp_out_thermal_.c_str());
  }
  if (ros::param::get("~tp_out_thermal_camerainfo", tp_out_thermal_camerainfo_)) {
    ROS_INFO("Retrived param 'tp_out_thermal_camerainfo': %s", tp_out_thermal_camerainfo_.c_str());
  }
  if (ros::param::get("~tp_out_depth_rgb", tp_out_depth_rgb_)) {
    ROS_INFO("Retrived param 'tp_out_depth_rgb': %s", tp_out_depth_rgb_.c_str());
  }
  if (ros::param::get("~tp_out_depth_thermal", tp_out_depth_thermal_)) {
    ROS_INFO("Retrived param 'tp_out_depth_thermal': %s", tp_out_depth_thermal_.c_str());
  }
  if (ros::param::get("~tp_out_intensity_rgb", tp_out_intensity_rgb_)) {
    ROS_INFO("Retrived param 'tp_out_intensity_rgb': %s", tp_out_intensity_rgb_.c_str());
  }
  if (ros::param::get("~tp_out_intensity_thermal", tp_out_intensity_thermal_)) {
    ROS_INFO("Retrived param 'tp_out_intensity_thermal': %s", tp_out_intensity_thermal_.c_str());
  }

  // 构建模块参数
  // ==========
  CommonFuns::load_intrinsic(intrinsic_rgb_, K_RGB_, D_RGB_);
  cv::eigen2cv(K_RGB_, cvK_RGB_);
  Ka_RGB_.block(0, 0, 3, 3) = K_RGB_;
  Ka_RGB_.col(3) = Eigen::Vector3f::Zero();
  K_RGB_inv_ = K_RGB_.inverse();
  cv::eigen2cv(D_RGB_, cvD_RGB_);

  CommonFuns::load_intrinsic(intrinsic_thermal_, K_Thermal_, D_Thermal_);
  cv::eigen2cv(K_Thermal_, cvK_Thermal_);
  Ka_Thermal_.block(0, 0, 3, 3) = K_Thermal_;
  Ka_Thermal_.col(3) = Eigen::Vector3f::Zero();
  K_Thermal_inv_ = K_Thermal_.inverse();
  cv::eigen2cv(D_Thermal_, cvD_Thermal_);

  CommonFuns::load_extrinsic(extrinsic_livox2rgb_, T_Livox2RGB_);
  T_Livox2RGB_inv_ = T_Livox2RGB_.inverse();
  cv::eigen2cv(T_Livox2RGB_, cvT_Livox2RGB_);

  CommonFuns::load_extrinsic(extrinsic_livox2thermal_, T_Livox2Thermal_);
  T_Livox2Thermal_inv_ = T_Livox2Thermal_.inverse();
  cv::eigen2cv(T_Livox2Thermal_, cvT_Livox2Thermal_);

  CommonFuns::load_extrinsic(extrinsic_thermal2rgb_, T_Thermal2RGB_);
  cv::eigen2cv(T_Thermal2RGB_, cvT_Thermal2RGB_);
  T_Thermal2RGB_inv_ = T_Thermal2RGB_.inverse();

  CommonFuns::load_homograph(homograph_thermal2rgb_, H_Thermal2RGB_);
  cv::eigen2cv(H_Thermal2RGB_, cvH_Thermal2RGB_);
  H_Thermal2RGB_inv_ = H_Thermal2RGB_.inverse();

  rgb_size_.width = rgb_width_; rgb_size_.height = rgb_height_;
  thermal_size_.width = thermal_width_; thermal_size_.height = thermal_height_;

  std::cout << "intrinsic_rgb " << std::endl;
  std::cout << "--------------" << std::endl;
  std::cout << "K: " << std::endl << K_RGB_ << std::endl;
  std::cout << "D: " << std::endl << D_RGB_ << std::endl << std::endl;

  std::cout << "intrinsic_thermal" << std::endl;
  std::cout << "-----------------" << std::endl;
  std::cout << "K: " << std::endl << K_Thermal_ << std::endl;
  std::cout << "D: " << std::endl << D_Thermal_ << std::endl << std::endl;

  std::cout << "extrinsic_livox2rgb" << std::endl;
  std::cout << "-------------------" << std::endl;
  std::cout << "T: " << std::endl << T_Livox2RGB_ << std::endl << std::endl;

  std::cout << "extrinsic_livox2thermal" << std::endl;
  std::cout << "-----------------------" << std::endl;
  std::cout << "T: " << std::endl << T_Livox2Thermal_ << std::endl << std::endl;

  std::cout << "extrinsic_thermal2rgb" << std::endl;
  std::cout << "---------------------" << std::endl;
  std::cout << "T: " << std::endl << T_Thermal2RGB_ << std::endl;

  std::cout << "homograph_thermal2rgb" << std::endl;
  std::cout << "---------------------" << std::endl;
  std::cout << "H: " << std::endl << H_Thermal2RGB_ << std::endl;

  // 创建发布主题
  // ==========
  pub_rgb_ = nh.advertise<sensor_msgs::Image>(tp_out_rgb_, 1);
  pub_rgb_camerainfo_ = nh.advertise<sensor_msgs::CameraInfo>(tp_out_rgb_camerainfo_, 1);
  pub_thermal_ = nh.advertise<sensor_msgs::Image>(tp_out_thermal_, 1);
  pub_thermal_camerainfo_ = nh.advertise<sensor_msgs::CameraInfo>(tp_out_thermal_camerainfo_, 1);
  pub_depth_rgb_ = nh.advertise<sensor_msgs::Image>(tp_out_depth_rgb_, 1);
  pub_depth_thermal_ = nh.advertise<sensor_msgs::Image>(tp_out_depth_thermal_, 1);
  pub_intensity_rgb_ = nh.advertise<sensor_msgs::Image>(tp_out_intensity_rgb_, 1);
  pub_intensity_thermal_ = nh.advertise<sensor_msgs::Image>(tp_out_intensity_thermal_, 1);

  // 主题订阅
  // =======
  image_transport::Subscriber sub_rgb ;
  image_transport::Subscriber sub_thermal ;
  message_filters::Subscriber<sensor_msgs::Image> sub_project_rgb(nh, tp_in_rgb_, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_project_thermal(nh, tp_in_thermal_, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_project_livox(nh, tp_in_livox_, 1);

  // RGB 修正后重新发布
  // ----------------
  if (!rgb_fuse_with_thermal_ && output_depth_on_rgb_)
  {
    sub_rgb = it.subscribe(tp_in_rgb_, 1, rgb_callback);
  }
  // Thermal 修正后重新发布
  // --------------------
  if (output_depth_on_thermal_)
  {
    sub_thermal = it.subscribe(tp_in_thermal_, 1, thermal_callback);
  }
  ros::Subscriber sub_livox = nh.subscribe(tp_in_livox_, 1, livox_callback);

  // RGB融合Thermal后重新发布
  // ----------------------
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_rgb_thermal;
  message_filters::Synchronizer<sync_rgb_thermal>
      sync_rgb_fuse_with_thermal(sync_rgb_thermal(100), sub_project_rgb, sub_project_thermal);
  if (rgb_fuse_with_thermal_) {
    // 设置回调函数
    sync_rgb_fuse_with_thermal.registerCallback(boost::bind(&rgb_fuse_with_thermal_callback, _1, _2));
  }

  ros::spin();

  return 0;
}

