/** @brief 3DBBox估算
 *
 *  给定参数文件标定相机-雷达，重新发布数据
 *
 * 数据采集说明
 * ==========
 * 1. 采数据主要用于调试使用(静态分析)
 * 2. 通过参数 data_acqu_path 配置，空-不采集， 非空-采集
 * 3. 采集数据格式参考 CommonFuns::save_data描述
 * 4. 文件命名方式： frame_index 取自 MOT::frame_index 由目标检测模块产生
 *    检测数据:  <data_acqu_path>/<frame_index>_data.xml
 *     深度图:  <data_acqu_path>/<frame_index>_depth.jpg
 *     彩色图:  <data_acqu_path>/<frame_index>_img.jpg
 *
 * 数据处理流程
 * ==========
 *  mot ---> MOT[多目标检测跟踪] ------------------\
 *  sensors_calib -+-> camera_info[CameraInfo] --|
 *                 |-> depth[Image] -------------o---> estimate_3dbbox ->
 * MOT[update]
 *                 \-> intensity[Image]----------|
 *  T_l2c ---------------------------------------|
 * T_l2w ---------------------------------------/
 *
 *  estimate_3ddbox
 *  ---------------
 *  depth -----o-> obj_depth -----o---> cloud -> filter ground -> filters ->
 * 欧氏分割 -> 3dbbox |                  | bbox  -----o                  | | |
 *  intensity--o-> obj_intensity -/
 *
 * Parameters:
 * ===========
 *  frame_id                -
 *  verbose                 - 调试等级
 *  pose_livox2world        - livox位姿, [x, y, z, R, P, Y]
 *  extrinsic_livox2rgb     - 雷达-相机外参文件
 *
 *  filter_ground_d_thr     - 地面过滤: 平面距离阈值,
 * 点与平面距离小于阈值都视为平面内点 filter_ground_n_thr     - 地面过滤:
 * 法线阈值, 平面法线大于阈值(coeff[2]>0.9)才被认为是地平面
 *  filter_ground_percent   - 地面过滤: 迭代剩余点百分比
 *
 *  sec_min_size            - 欧氏分割:
 * 最少点数，//euclidean_cluster.setMinClusterSize(cluster_size_min_)
 *  sec_max_size            - 欧氏分割:
 * 最多点数，//euclidean_cluster.setMaxClusterSize(cluster_size_max_) sec_coeff
 * - 欧氏分割: 阈值系数
 *
 *  w_points = 0.3;         - 模板匹配选取目标区域: 点云点数权重
 *  w_distance = 0.3;       - 模板匹配选取目标区域: 距离权重
 *  w_matching = 0.4;       - 模板匹配选取目标区域: 匹配度权重
 *  gather_coef = 1.0;      - 模板匹配选取目标区域: 聚合阈值系数
 *  pose_type = 0;          - 模板匹配选取目标区域: 位姿估算方法,
 * 0-由PCA估算，1-由聚合点云边界确定,2-聚类首元点云边界 norm_type = 0; -
 * 模板匹配选取目标区域: 统计量归一化方法, 0-最大值比值法，1-总数比值法,详细看
 * obj_cloud_pattern.h data_acqu_path          -
 * 数据采集保存目录，空-不采集，非空-采集，默认空，
 * 采集的数据主要用于调试(静态分析) e.g. /tmp/data_acqu
 *
 *  tp_in_depth             - 订阅主题: 深度图
 *  tp_in_depth_camerainfo  - 订阅主题: 深度图相机参数
 *  tp_in_intensity         - 订阅主题: 强度值图
 *  tp_in_mot               - 订阅主题: 多目标检测/跟踪
 *  tp_out_mot              - 发布主题: 多目标检测/跟踪
 *
 * Subscribe
 * ==========
 *  多目标检测跟踪[mot/MOT]
 *    /mosif/mot/MOT
 *  深度图相机内参[sensor_msgs/CameraInfo]
 *    /libox/depth_camerainfo
 *  深度图[sensors_msg/Image]
 *    /livox/depth
 *
 *
 * Publish
 * ========
 *  多目标检测跟踪[mot/MOT]
 *    /mot/MOT
 *
 * Verbose[debug]
 * ==============
 *   verbose=2
 *   /dbg/objs_3dbbox[sensor_msgs/PointCloud2] -
 * 检测目标3dbbox估算，由dbg_pub_objs_3dbbox_发布
 *
 */
#include "../common/estimate_3dbbox.h"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/Marker.h>
#include <mot/MOT.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>  //  //PCL可视化的头文件
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <tf/transform_listener.h>

#include <Eigen/Eigen>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>

#include "../common/common_view.h"

/** 参数配置
 */
std::string frame_id_;
int verbose_ = 0;                   // 特别： (100)保存数据到 /tmp/data
std::string pose_livox2world_str_;  // 套件安装位姿
std::string extrinsic_livox2rgb_;   // 外参标定文件
std::string tp_in_depth_;           // 订阅主题: 深度图相机
std::string tp_in_depth_camerainfo_;  // 订阅主题: 深度图相机参数
std::string tp_in_intensity_;         // 订阅主题: 强度值图
std::string tp_in_mot_;               // 订阅主题: 多目标检测与跟踪
std::string tp_out_mot_;  // 发布主题: 多目标检测与跟踪(更新目标位姿信息)
float filter_ground_d_thr_ =
    0.1;  // 地面过滤: 平面距离阈值, 点与平面距离小于阈值都视为平面内点
float filter_ground_n_thr_ =
    0.9;  // 地面过滤: 法线阈值, 平面法线大于阈值(coeff[2]>0.9)才被认为是地平面
float filter_ground_percent_ = 0.3;  // 地面过滤: 迭代剩余点百分比
int sec_min_size_ =
    10;  // 欧氏分割:
         // 最少点数，//euclidean_cluster.setMinClusterSize(cluster_size_min_)
int sec_max_size_ =
    3000;  // 欧氏分割:
           // 最多点数，//euclidean_cluster.setMaxClusterSize(cluster_size_max_)
float sec_coeff_ = 40.0;   // 欧氏分割: 邻近点阈值系数
float w_matching_ = 0.6;   // 模板匹配选取目标区域: 匹配度权重
float w_distance_ = 0.2;   // 模板匹配选取目标区域: 距离权重
float w_points_ = 0.2;     // 模板匹配选取目标区域: 点云点数权重
float gather_coef_ = 1.0;  // 模板匹配选取目标区域: 聚合阈值系数
int pose_type_ = 0;  // 模板匹配选取目标区域: 位姿估算方法,
                     // 0-由PCA估算，1-由聚合点云边界确定,2-聚类首元点云边界
int norm_type_ = 0;  // 模板匹配选取目标区域: 统计量归一化方法,
                     // 0-最大值比值法，1-总数比值法,详细看 obj_cloud_pattern.h
std::string
    data_acqu_path_;  // 数据采集保存目录，空-不采集，非空-采集，默认空，
                      // 采集的数据主要用于调试(静态分析) e.g. /tmp/data_acqu

/** 订阅/发布主题
 */
ros::Subscriber sub_depth_camerainfo_;  // 订阅深度相机参数
ros::Publisher pub_mot_;  // 发布多目标检测/跟踪(更新3dbbox后重新发布)
ros::Publisher dbg_pub_objs_3dbbox_;  // 调试发布主题: 绘制3dbbox后的点云数据

/** 调试数据
 */
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_(new
// pcl::visualization::PCLVisualizer("3D Viewer"));
pcl::visualization::PCLVisualizer::Ptr viewer_ =
    NULL;  //(new pcl::visualization::PCLVisualizer("3D Viewer"));
cv::Mat dbg_depth_;
std::vector<Eigen::Matrix<float, 1, 9>> dbg_bbox3ds_;
pcl::PointCloud<pcl::PointXYZ>::Ptr dbg_cloud_(
    new pcl::PointCloud<pcl::PointXYZ>());

/** 模块参数
 */
bool ready_ = false;              // 参数准备好
Estimate3DBBox estimate_3dbbox_;  // 3DBBOX估算模块
int frame_index_ = 0;             // 数据帧序号

/** @brief 数据提取
 * @param msg in - mot::MOT
 * @param class_ids out -  目标类别ID序列
 * @param class_names out - 目标类别名称序列
 * @param bboxes out - 目标BBOX序列[(top, left, bottom, right),...]
 * @param scores out - 目标置信度序列
 * @param track_ids out - 目标跟踪ID序列
 */
void extract_data(const mot::MOT::ConstPtr msg, std::vector<int> &class_ids,
                  std::vector<std::string> &class_names,
                  std::vector<int> &bboxes, std::vector<float> &scores,
                  std::vector<int> &track_ids) {
  for (unsigned int i = 0; i < msg->objs.size(); i++) {
    class_ids.push_back(msg->objs[i].class_id);
    class_names.push_back(msg->objs[i].class_name);
    for (unsigned int j = 0; j < 4; j++) {
      bboxes.push_back(msg->objs[i].bbox[j]);
    }
    scores.push_back(msg->objs[i].score);
    track_ids.push_back(msg->objs[i].track_id);
  }
}

/** @brief 深度相机参数订阅
 */
void depth_camera_info_callback(const sensor_msgs::CameraInfo &msg) {
  if (msg.K.size() <= 0 || ready_) {
    return;
  }
  Eigen::Matrix<float, 3, 3> K;
  K << msg.K[0], msg.K[1], msg.K[2], msg.K[3], msg.K[4], msg.K[5], msg.K[6],
      msg.K[7], msg.K[8];
  estimate_3dbbox_.set_K(K);

  ready_ = true;

  std::cout << "K:" << std::endl << K << std::endl << std::endl;
}

/** @brief 深度图订阅
 */
tf::TransformListener *tf_listener;
tf::StampedTransform base_to_map;
void depth_callback(const sensor_msgs::Image::ConstPtr depth_msg,
                    mot::MOT::ConstPtr mot_msg) {
  // depth_msg => depth
  //  /usr/bin/ld: warning: libopencv_imgcodecs.so.3.2, needed by
  //  /opt/ros/melodic/lib/libcv_bridge.so, may conflict with
  //  libopencv_imgcodecs.so.3.4 /usr/bin/ld: warning: libopencv_core.so.3.4,
  //  needed by /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.4.1, may
  //  conflict with libopencv_core.so.3.2
  cv::Mat depth = cv_bridge::toCvShare(depth_msg, "32FC1")->image;
  // cv::Mat intensity = cv_bridge::toCvShare(intensity_msg, "32FC1")->image;
  cv::Mat intensity(depth.rows,depth.cols,CV_32F,cv::Scalar(1.0));
  // 提取数据
  std::vector<int> class_ids;
  std::vector<std::string> class_names;
  std::vector<int> bboxes;
  std::vector<float> scores;
  std::vector<int> track_ids;
  extract_data(mot_msg, class_ids, class_names, bboxes, scores, track_ids);

  // 3DBBox估算
  std::vector<Eigen::Matrix<float, 1, 9>> bbox3ds;
  // bbox3ds.resize(mot_msg->objs.size());
  tf_listener->lookupTransform("/map", "/fl_cam_color_optical_frame",
                               ros::Time(0), base_to_map);
  Eigen::Matrix4f camera_to_map;
  pcl_ros::transformAsMatrix(base_to_map, camera_to_map);
  estimate_3dbbox_.set_T_l2c(Eigen::MatrixXf::Identity(4, 4));
  estimate_3dbbox_.set_T_l2w(camera_to_map);
  estimate_3dbbox_.estimate_3dbboxs(depth, intensity, class_ids, class_names,
                                    bboxes, scores, track_ids, bbox3ds);

  //  // 输出调试: 在PCL视图里绘制3DBBox！
  //  注:因为没有找到直接在点云里绘制的方案，所以只能在viewer容器里绘制. if
  //  (verbose_ > 1 && viewer_) {
  //    printf("[D:%s:%d][depth_callback] \n", __FILE__, __LINE__);
  //    CommonFuns::depth2cloud(depth, dbg_cloud_,
  //                            estimate_3dbbox_.get_K_inv(),
  //                            estimate_3dbbox_.get_T_l2c_inv(),
  //                            estimate_3dbbox_.get_T_l2w());
  //    viewer_->updatePointCloud(dbg_cloud_, "dbg_cloud");
  //    viewer_->removeAllShapes();
  //    for (unsigned int i = 0; i< bbox3ds.size(); i++) {
  //      char id[255] = {0};
  //      sprintf(id, "cloud1_%d", i);
  //      std::cout << "bbox3d: " << bbox3ds[i] << std::endl;
  //      CommonView::draw_3dbbox(viewer_, bbox3ds[i], 255<<16, id, 0);
  //    }
  //  }

  // 输出调试: 数据采集 保存到 /tmp/data
  if (data_acqu_path_ != "") {
    cv::Mat bgr8 = cv_bridge::toCvCopy(mot_msg->image, "bgr8")->image;
    CommonFuns::save_data(data_acqu_path_.c_str(), mot_msg->frame_index, bgr8,
                          depth, intensity, class_ids, class_names, bboxes,
                          scores, track_ids);
    printf("[D:%s:%d] save to %s: %d\n", __FILE__, __LINE__,
           data_acqu_path_.c_str(), mot_msg->frame_index);
  }

  // 重新发布 mot
  if (1) {
    mot::MOT::Ptr mot(new mot::MOT());
    //  拷贝原内容
    mot->header = mot_msg->header;
    if (frame_id_ != "") {
      mot->header.frame_id = frame_id_;
    }
    mot->image = mot_msg->image;
    mot->frame_index = mot_msg->frame_index;
    mot->objs = mot_msg->objs;
    //  更新内容: 3dbbox
    for (unsigned i = 0; i < mot->objs.size(); i++) {
      mot->objs[i].bbox3d.clear();
      for (unsigned int j = 0; j < 9; j++) {
        mot->objs[i].bbox3d.push_back(bbox3ds[i][j]);
      }
    }
    pub_mot_.publish(mot);
  }

  frame_index_++;
  if (verbose_ > 0) {
    printf("[D:%s:%d] frame_index=%d\n", __FILE__, __LINE__, frame_index_);
  }
}

/** @brief 主函数
 */
int main(int argc, char **argv) {
  // ROS节点初始化
  // ===========
  ros::init(argc, argv, "estimate_3dbbox");
  ROS_INFO("estimate_3dbbox initializtion!");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  tf_listener = new tf::TransformListener();
  // 读取ROS参数
  // ==========
  if (ros::param::get("~frame_id", frame_id_)) {
    ROS_INFO("Retrived param 'frame_id': %s", frame_id_.c_str());
  }
  if (ros::param::get("~verbose", verbose_)) {
    ROS_INFO("Retrived param 'verbose': %d", verbose_);
  }
  if (ros::param::get("~pose_livox2world", pose_livox2world_str_)) {
    ROS_INFO("Retrived param 'pose_livox2world': %s",
             pose_livox2world_str_.c_str());
  }
  if (ros::param::get("~extrinsic_livox2rgb", extrinsic_livox2rgb_)) {
    ROS_INFO("Retrived param 'extrinsic_livox2rgb': %s",
             extrinsic_livox2rgb_.c_str());
  }
  if (ros::param::get("~filter_ground_d_thr", filter_ground_d_thr_)) {
    ROS_INFO("Retrived param 'filter_ground_d_thr': %f", filter_ground_d_thr_);
  }
  if (ros::param::get("~filter_ground_n_thr", filter_ground_n_thr_)) {
    ROS_INFO("Retrived param 'filter_ground_n_thr': %f", filter_ground_n_thr_);
  }
  if (ros::param::get("~filter_ground_percent", filter_ground_percent_)) {
    ROS_INFO("Retrived param 'filter_ground_percent': %f",
             filter_ground_percent_);
  }
  if (ros::param::get("~sec_coeff", sec_coeff_)) {
    ROS_INFO("Retrived param 'sec_coeff': %f", sec_coeff_);
  }
  if (ros::param::get("~sec_min_size", sec_min_size_)) {
    ROS_INFO("Retrived param 'sec_min_size': %d", sec_min_size_);
  }
  if (ros::param::get("~sec_max_size", sec_max_size_)) {
    ROS_INFO("Retrived param 'sec_max_size': %d", sec_max_size_);
  }
  if (ros::param::get("~w_matching", w_matching_)) {
    ROS_INFO("Retrived param 'w_matching': %f", w_matching_);
  }
  if (ros::param::get("~w_distance", w_distance_)) {
    ROS_INFO("Retrived param 'w_distance': %f", w_distance_);
  }
  if (ros::param::get("~w_points", w_points_)) {
    ROS_INFO("Retrived param 'w_points': %f", w_points_);
  }
  if (ros::param::get("~gather_coef", gather_coef_)) {
    ROS_INFO("Retrived param 'gather_coef': %f", gather_coef_);
  }
  if (ros::param::get("~pose_type", pose_type_)) {
    ROS_INFO("Retrived param 'pose_type': %d", pose_type_);
  }
  if (ros::param::get("~norm_type", norm_type_)) {
    ROS_INFO("Retrived param 'norm_type': %d", norm_type_);
  }
  if (ros::param::get("~data_acqu_path", data_acqu_path_)) {
    ROS_INFO("Retrived param 'data_acqu_path': %s", data_acqu_path_.c_str());
    // 自动创建目录
    if (data_acqu_path_ != "") {
      char cmd[255] = {0};
      sprintf(cmd, "mkdir -p %s", data_acqu_path_.c_str());
      ROS_INFO("CMD: %s", cmd);
      system(cmd);
    }
  }
  if (ros::param::get("~tp_in_depth", tp_in_depth_)) {
    ROS_INFO("Retrived param 'tp_in_depth': %s", tp_in_depth_.c_str());
  }
  if (ros::param::get("~tp_in_depth_camerainfo", tp_in_depth_camerainfo_)) {
    ROS_INFO("Retrived param 'tp_in_depth_camerainfo': %s",
             tp_in_depth_camerainfo_.c_str());
  }
  if (ros::param::get("~tp_in_intensity", tp_in_intensity_)) {
    ROS_INFO("Retrived param 'tp_in_intensity': %s", tp_in_intensity_.c_str());
  }
  if (ros::param::get("~tp_in_mot", tp_in_mot_)) {
    ROS_INFO("Retrived param 'tp_in_mot': %s", tp_in_mot_.c_str());
  }
  if (ros::param::get("~tp_out_mot", tp_out_mot_)) {
    ROS_INFO("Retrived param 'tp_out_mot': %s", tp_out_mot_.c_str());
  }

  // 构建模块参数
  // ==========
  // 调试等级
  estimate_3dbbox_.set_verbose(verbose_);
  // 坐标体系: 相机内参由订阅深度相机参数主题设置
  // 坐标体系: 雷达-相机外参
  estimate_3dbbox_.load_extrinsic_l2c(extrinsic_livox2rgb_);
  // 坐标体系: 相机套件(Livox坐标系)位姿，参考系为世界坐标系
  estimate_3dbbox_.load_extrinsic_l2w(pose_livox2world_str_);
  // 地面过滤参数
  estimate_3dbbox_.set_filter_ground(filter_ground_d_thr_, filter_ground_n_thr_,
                                     filter_ground_percent_);
  // 欧氏分割参数
  estimate_3dbbox_.set_segment_euclidean_cluster(sec_coeff_, sec_min_size_,
                                                 sec_max_size_);
  // 模板匹配选择目标区域参数
  estimate_3dbbox_.set_obj_cloud_pattern_params(w_matching_, w_distance_,
                                                w_points_, gather_coef_,
                                                pose_type_, norm_type_);

  std::cout << "extrinsic_livox2rgb " << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << "T_l2c: " << std::endl
            << estimate_3dbbox_.get_T_l2c() << std::endl;
  std::cout << "pose_livox2world" << std::endl;
  std::cout << "----------------" << std::endl;
  std::cout << "T_l2w: " << std::endl
            << estimate_3dbbox_.get_T_l2w() << std::endl;

  // 创建发布主题
  // ==========
  std::cout << "generate publisher mot with " << tp_out_mot_ << std::endl;
  pub_mot_ = nh.advertise<mot::MOT>(tp_out_mot_, 1);

  // 主题订阅
  // =======
  // 订阅深度相机参数
  // --------------
  std::cout << "generate subscriber depth_camera_info with "
            << tp_in_depth_camerainfo_ << std::endl;
  ros::Subscriber sub_depth_camerainfo =
      nh.subscribe(tp_in_depth_camerainfo_, 1, depth_camera_info_callback);

  // 同步订阅深度图和多目标检测跟踪
  // -------------------------
  std::cout << "generate message_filters depth and mot with " << tp_in_depth_
            << " and " << tp_in_mot_ << std::endl;
  // refer to message_filters: http://wiki.ros.org/message_filters
  message_filters::Subscriber<sensor_msgs::Image> sub_depth(nh, tp_in_depth_,
                                                            1);
  // message_filters::Subscriber<sensor_msgs::Image> sub_intensity(
  //     nh, tp_in_intensity_, 1);
  message_filters::Subscriber<mot::MOT> sub_mot(nh, tp_in_mot_, 1);
  //  // ApproximateTime takes a queue size as its constructor argument, hence
  //  MySyncPolicy(10)
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          mot::MOT>
      sync;
  message_filters::Synchronizer<sync> sync_3dbbox(sync(10), sub_depth, sub_mot);
  // 设置回调函数
  sync_3dbbox.registerCallback(boost::bind(&depth_callback, _1, _2));

  //  // 点云显示
  //  // =======
  //  if (verbose_>1) {
  //    viewer_ = pcl::visualization::PCLVisualizer::Ptr(new
  //    pcl::visualization::PCLVisualizer("3D Viewer"));
  //    viewer_->initCameraParameters(); // 多窗口显示需要，否则无效
  //    viewer_->addCoordinateSystem(3.0);
  //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_empty(new
  //    pcl::PointCloud<pcl::PointXYZ>()); viewer_->addPointCloud(cloud_empty,
  //    "dbg_cloud"); while (!viewer_->wasStopped())
  //    {
  //      ros::spinOnce();
  //      viewer_->spinOnce(100);
  //      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  //    }
  //  }
  while (nh.ok()) {
    try {
      tf_listener->waitForTransform("/map", "/fl_cam_color_optical_frame",
                                    ros::Time(0), ros::Duration(3.0));
      tf_listener->lookupTransform("/map", "/fl_cam_color_optical_frame",
                                   ros::Time(0), base_to_map);
      break;
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
  ros::spin();

  return 0;
}
