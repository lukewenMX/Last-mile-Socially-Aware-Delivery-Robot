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
 * Parameters:
 * ===========
 *  frame_id                  - ROS Message header.frame_id
 *  verbose                   - 日志输出等级
 *  # output_depth_on_rgb, output_depth_on_thermal, rgb_fuse_with_thermal 控制输出模式
 *  intrinsic_rgb             - 彩色相机内参文件, default: <sensors_calib>/config/intrinsic_rgb.txt
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
 *  min_distance              - 最小测距(livox), default: 1.0m
 *  max_distance              - 最大测距(livox), default: 300.0m
 *  tp_in_rgb                 - 彩色相机订阅主题, default: /rgb_cam/image_raw
 *  tp_in_rgb_camerainfo      - 彩色相机参数订阅主题, default: /rgb_cam/camerainfo
 *  tp_out_rgb                - 彩色相机重新发布主题, default: /rgb_cam/image_raw
 *  tp_out_rgb_camerainfo     - 彩色相机参数重新发布主题, default: /rgb_cam/camerainfo
 *  tp_out_depth_thermal      - 深度图发布主题(基于热感相机参数), default: /livox/depth_thermal
 *  tp_out_intensity_rgb      - 强度值图发布主题(基于彩色相机参数): default: /livox/intensity_rgb
 *
 *  说明: 相机图像主题以 /compressed 结束时为压缩格式
 *
 * Subscribe
 * ==========
 *  彩色相机图像[sensors_msg/Image | sensor_msgs/CompressedImage]
 *    /rgb_cam/image_raw or /rgb_cam/image_raw/compressed
 *  彩色相机参数[sensor_msgs/CameraInfo]
 *    /rgb_cam/camerainfo
 *
 *  说明:真实主题路径以实际设置为准
 *
 * Publish
 * ========
 *  彩色相机图像[sensors_msg/Image | sensor_msgs/CompressedImage]
 *    /rgb_cam/image_raw or /rgb_cam/image_raw/compressed
 *  彩色相机参数[sensor_msgs/CameraInfo]
 *    /rgb_cam/camerainfo
 *  强度值图(基于彩色相机参数)[sensor_msgs/Image, float32]
 *    /livox/intensity_rgb
 *
 *  说明:真实主题路径以实际设置为准
 *
 * Verbose[debug]
 * ==============
 *   verbose=2
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
std::string intrinsic_rgb_;             // 彩色相机内参文件
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
std::string tp_out_rgb_;                // 发布主题: 彩色相机
std::string tp_out_rgb_camerainfo_;     // 发布主题: 彩色相机内参
std::string tp_out_intensity_rgb_;      // 发布主题: 强度图(基于彩色相机参数)

/** 订阅/发布主题
 */
ros::Subscriber sub_rgb_;                 // 订阅:彩色图像
ros::Subscriber sub_rgb_camerainfo_;      // 订阅:彩色相机参数
ros::Publisher  pub_rgb_;                 // 发布:彩色图像
ros::Publisher  pub_rgb_camerainfo_;      // 发布:彩色相机参数
ros::Publisher  pub_intensity_rgb_;       // 发布:强度图像(基于彩色相机参数)


/** 相机-雷达内外参他
 */
Eigen::Matrix3f             K_RGB_;               // 彩色相机内参
Eigen::Matrix3f             K_RGB_inv_;           // 彩色相机内参逆
Eigen::Matrix<float, 3, 4>  Ka_RGB_;              // 彩色相机内参:增广矩阵， 最右列为0
Eigen::Matrix<float, 1, 5>  D_RGB_;               // 彩色相机畸变参数


cv::Mat  cvK_RGB_;              // 彩色相机内参: 3x3
cv::Mat  cvD_RGB_;              // 彩色相机畸变参数: 1x5


cv::Size rgb_size_(rgb_width_, rgb_height_);              // RGB图像尺寸


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

/** @brief 生成伪造深度图像
 *  1。创建与rgb图像大小一直的强度值
 *  2。发布主题，强度值图
 */
void publish_intensity(const sensor_msgs::ImageConstPtr& msg,
                       const cv::Size & cam_size,
                       ros::Publisher &pub_intensity)
{
  //生成rgb图像尺寸大小的空强度值图
  cv::Mat intensity = cv::Mat::zeros(cam_size.height, cam_size.width, CV_32FC1);
  //转换ROS格式
  sensor_msgs::ImagePtr ros_intensity = cv_bridge::CvImage(msg->header, "32FC1", intensity).toImageMsg();

  if (frame_id_ !=""){
    ros_intensity->header.frame_id = frame_id_;
  }

  //发布强度值图
  pub_intensity.publish(ros_intensity);
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
  // 发布伪造强度值图
  publish_intensity(msg, rgb_size_, 
                         pub_intensity_rgb_);
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
  if (ros::param::get("~intrinsic_rgb", intrinsic_rgb_)) {
    ROS_INFO("Retrived param 'intrinsic_rgb': %s", intrinsic_rgb_.c_str());
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
  if (ros::param::get("~tp_out_rgb", tp_out_rgb_)) {
    ROS_INFO("Retrived param 'tp_out_rgb': %s", tp_out_rgb_.c_str());
  }
  if (ros::param::get("~tp_out_rgb_camerainfo", tp_out_rgb_camerainfo_)) {
    ROS_INFO("Retrived param 'tp_out_rgb_camerainfo': %s", tp_out_rgb_camerainfo_.c_str());
  }
  if (ros::param::get("~tp_out_intensity_rgb", tp_out_intensity_rgb_)) {
    ROS_INFO("Retrived param 'tp_out_intensity_rgb': %s", tp_out_intensity_rgb_.c_str());
  }

  // 构建模块参数
  // ==========
  CommonFuns::load_intrinsic(intrinsic_rgb_, K_RGB_, D_RGB_);
  cv::eigen2cv(K_RGB_, cvK_RGB_);
  Ka_RGB_.block(0, 0, 3, 3) = K_RGB_;
  Ka_RGB_.col(3) = Eigen::Vector3f::Zero();
  K_RGB_inv_ = K_RGB_.inverse();
  cv::eigen2cv(D_RGB_, cvD_RGB_);


  rgb_size_.width = rgb_width_; rgb_size_.height = rgb_height_;

  std::cout << "intrinsic_rgb " << std::endl;
  std::cout << "--------------" << std::endl;
  std::cout << "K: " << std::endl << K_RGB_ << std::endl;
  std::cout << "D: " << std::endl << D_RGB_ << std::endl << std::endl;


  // 创建发布主题
  // ==========
  pub_rgb_ = nh.advertise<sensor_msgs::Image>(tp_out_rgb_, 1);
  pub_rgb_camerainfo_ = nh.advertise<sensor_msgs::CameraInfo>(tp_out_rgb_camerainfo_, 1);
  pub_intensity_rgb_ = nh.advertise<sensor_msgs::Image>(tp_out_intensity_rgb_, 1);


  // 主题订阅
  // =======
  image_transport::Subscriber sub_rgb = it.subscribe(tp_in_rgb_, 1, rgb_callback) ;



  ros::spin();

  return 0;
}

