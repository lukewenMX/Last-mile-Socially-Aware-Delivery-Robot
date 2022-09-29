#ifndef ESTIMATE_3DBBOX_H
#define ESTIMATE_3DBBOX_H

/** @brief 3DBBox估算封装类
 *
 *  estimate_3ddbox
 *  ---------------
 *  depth -o-> obj_depth -> cloud -> filter ground -> filters -> 欧氏分割 -> 3dbbox
 *  bbox  -/
 *
 * 类别信息配置
 * ==========
 *  yaml文件格式
 *  -----------
 *    filter_class_ids: [<class_id1>, <class_id2>, ...]
 *    classes: { <class_id>: {'id:0, 'en_name: 'persion', 'cn_name: '人', 'size: 0., 'conf_th: 0.3, 'iou_th: 0.5}, ...}}
 *
 * 字段解析
 * -------
 *   filter_class_ids  - 检测目标过滤后留下的类别配置项, 空表示不过滤
 *   {<class_id>:{'id:0, 'en_name: 'persion', 'cn_name: '人', 'size: [0.,0.,0.], 'conf_th: 0.3, 'iou_th: 0.5}, ...}
 *       class_id: 类别ID, 注{每个模型有自己ID规则}
 *       en_name:  类别英文名称
 *       cn_name:  类别中文名称
 *       size：    目标尺寸(长(x)宽(y)高(z))， 0.表示未知，由检测结果估算碰撞区域半径； 否则按给定值设置碰撞区域半径.
 *       conf_th:  置信度阈值， 0-1之间
 *       iou_th:   IoU阈值，0-1之间 ； 保留参数， 忽略
 *       icon: 雷达图显示图标文件，空默认实心小圆圈；起始字符 / 为绝对路径， <pack_name>表示包命令, e.g.: /path/car.png, <crash_warning>/resource/car.png
 *       warn_level: 预警等级，数字越大等级越高, 一般物体: 0, 人-1, 动物-2, 车-3，飞机-4
 *                  0-指固定无法移动的物体，如建筑物，树等.
 *                  1-指一般无生命不能自主运动但容易受外力作用运动的物体，如球、悬挂物
 *                  2-能自主运动并能灵活避障的生物或移动工具，如人，鸟，自行车
 *                  3-能自主运动但容易手惊吓的生物或移动工具，如宠物、家禽、畜牧
 *                  4-小型汽车
 *                  5-大型汽车/卡车
 *                  6-飞机
 * 范例:
 *  <config.yaml>
 * --------------
 *  filter_class_ids: [0, 1, 2]
 *  classes:
 *    0:
 *      id: 0
 *      en_name: persion
 *      cn_name: 人
 *      size: [0.5, 0.5, 1.7]
 *      conf_th: 0.3
 *      iou_th: 0.5
 *      icon: ''
 *      warn_level: 2
 *    1:
 *      ...
 *      ...
 *      ...
 *
 * 使用范例
 * =======
 * // 定义3DBBox估算对象
 * // ----------------
 * Estimate3DBBox estimate_3dbbox();
 *
 * // 设置参数
 * // -------
 * // 调试等级
 * estimate_3dbbox.set_verbose(3);
 * // 地面过滤参数
 * estimate_3dbbox.set_filter_ground(param.filter_ground_d_thr, param.filter_ground_n_thr, param.filter_ground_percent);
 * // 欧氏分割参数
 * estimate_3dbbox.set_segment_euclidean_cluster(param.coeff, param.sec_min_size, param.sec_max_size);
 *
 * // 坐标体系: 相机内参
 * Eigen::Matrix3f K;
 * Eigen::Matrix<float, 1, 5> D;
 * CommonFuns::load_intrinsic(param.intrinsic_file, K, D);
 * estimate_3dbbox.set_K(K);
 * // 坐标体系: 雷达-相机外参
 * estimate_3dbbox.load_extrinsic_l2c(param.extrinsic_file);
 * // 坐标体系: 相机套件(Livox坐标系)位姿，参考系为世界坐标系
 * estimate_3dbbox.load_extrinsic_l2w(param.livox2world_pose);
 * // 模板匹配选择目标区域参数
 * estimate_3dbbox.set_obj_cloud_pattern_params(param.w_matching, param.w_distance, param.w_points,
 *                                              param.gather_coef, param.pose_type, param.norm_type);
 *
 * // 加载测试数据
 * // ----------
 * cv::Mat img;
 * cv::Mat depth;
 * cv::Mat intensity;
 * std::vector<int> class_ids ;
 * std::vector<std::string> class_names;
 * std::vector<int> bboxes;
 * std::vector<float> scores;
 * std::vector<int> track_ids;
 * CommonFuns::read_data(param.data_path, param.frame_index, img, depth, intensity,
 *                               class_ids, class_names, bboxes, scores, track_ids);
 *
 * // 3DBBOX估算
 * // ---------
 * std::vector<Eigen::Matrix<float, 1, 9>> bbox3ds;
 * estimate_3dbbox.estimate_3dbboxs(depth, intensity, class_ids, class_names, bboxes, scores, track_ids, bbox3ds);
 *
 *
 * 更新记录
 * =======
 * version: 0.0.1
 * version: 0.0.2
 *  添加强度值接口
 * version: 0.0.3
 *  添加地面过滤方法
 *  欧氏分割 tole 参数自适应(以person测试)
 *
 */
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <assert.h>

#include <Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "common.h"
#include "obj_cloud_pattern.h"

/** @brief 3DBBox估算封装类
 */
class Estimate3DBBox
{
public:
  // 数据处理参数
  typedef struct _Params
  {
    // 强度值滤波
    struct
    {
      float thr_val = 40;
    }intensity;

    // 地面过滤
    struct
    {
      float d_threshold = 0.1;   // 平面距离阈值: 点与平面距离小于阈值都视为平面内点
      float n_threshold = 0.9;    // 法线阈值: 平面法线大于阈值(coeff[2]>0.9)才被认为是地平面
      float percent = 0.2;        // 迭代剩余点百分比
      int min_size = 20;          // 最少点数
    }filter_ground;

    // 欧氏分割
    struct
    {
      int min_size = 10;    //euclidean_cluster.setMinClusterSize(cluster_size_min_); // 返回聚类过滤：最小点数
      int max_size = 3000;  //euclidean_cluster.setMaxClusterSize(cluster_size_max_); // 返回聚类过滤：最多点数
      float coeff = 40;     //邻近点阈值系数
    }sec;

    // 模板匹配选择目标区域参数
    struct
    {
      float w_points;     // 点云点数权重
      float w_distance;   // 距离权重
      float w_matching;   // 匹配度权重
      float gather_coef;  // 聚合阈值系数
      int   pose_type;    // 位姿估算方法， 0-由PCA估算(模板匹配法)， 1-由聚合点云边界确定(模板匹配法，忽略姿态), 2-简单聚类首元
      int   norm_type;    // 统计量正则化方法： 0-最大值比值法， 1-总和比值法；详细解析看 obj_cloud_patter.h
    }obj_cloud_pattern;

  }Params;

  // 类别信息
  typedef struct
  {
    int id;               //    id: 64
    std::string en_name;  //    en_name: 'mouse'
    std::string cn_name;  //    cn_name: '鼠标'
    float size[3];        //    size: [L, W, H]
    float conf_th;        //    conf_th: 0.3
    float iou_th;         //    iou_th: 0.5
    std::string icon;     //    icon:
    int warn_level;       //    warn_level: 0
  }ClassInfo;

public:
  Estimate3DBBox(int verbose=0);
  ~Estimate3DBBox();

public:
  void set_verbose(int verbose);            // 设置调试等级
  int  get_verbose();                       // 获取调试等级
  void set_K(Eigen::Matrix3f &K);           // 设置相机内参
  const Eigen::Matrix3f &get_K();           // 获取相机内参
  const Eigen::Matrix3f &get_K_inv();       // 获取相机内参的逆
  void set_T_l2c(const Eigen::Matrix4f &T);       // 设置雷达-相机外参
  const Eigen::Matrix4f &get_T_l2c();       // 获取雷达-相机外参
  const Eigen::Matrix4f &get_T_l2c_inv();   // 获取雷达-相机外参的逆
  void set_T_l2w(const Eigen::Matrix4f &T);       // 设置雷达位姿齐次矩阵:参考坐标是世界坐标
  const Eigen::Matrix4f &get_T_l2w();       // 获取雷达位姿齐次矩阵:参考坐标是世界坐标
  const Eigen::Matrix4f &get_T_l2w_inv();   // 获取雷达位姿齐次矩阵的逆:参考坐标是世界坐标
  void set_intensity(float thr_val);        // 设置强度值滤波参数
  void set_filter_ground(float d_threshold, float n_threshold, float percent);  // 设置地面过滤参数
  void set_segment_euclidean_cluster(float coeff, int min_size, int max_size);   // 设置欧氏分割参数
  bool load_extrinsic_l2c(const std::string &sfile);                // 加载雷达-相机外参
  bool load_extrinsic_l2w(const std::string &pose);                 // 加载雷达位姿
  bool load_class_info(const std::string &cfg_file);                // 加载类别配置文件
  bool get_class_info(int class_id, ClassInfo &info);               // 获取类别信息
  void set_sec_coeff(float coeff){param_.sec.coeff=coeff;}
  void set_obj_cloud_pattern_params(
      float w_matching, float w_distance, float w_points,
      float gather_coef, int pose_type, int norm_type);   // 设置模板匹配选择目标区域参数
  void get_obj_cloud_pattern_params(
      float &w_matching, float &w_distance, float &w_points,
      float &gather_coef, int &pose_type, int &norm_type); // 获取模板匹配选择目标区域参数

public:
  float caculate_sec_tole(int img_w, float fov, float divergence,
                          float obj_r, int obj_w, float coeff); // 计算欧氏分割邻近点阈值
  void extract_obj(const cv::Mat &depth, const cv::Mat &intensity, const Eigen::Vector4i &bbox,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr &points);    // 提取目标深度图区域,返回点云
  void filter_ground(int obj_index, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter); // 过滤地面
  void filter_noise(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);     // 噪声过滤
  Eigen::Matrix<float, 1, 9> get_3dbbox(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                                        const pcl::PointIndices &indices,
                                        int &num_points, float &distance);  // 获取点云3dbbox位姿

  Eigen::Matrix<float, 1, 9> estimate_3dbbox(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                             const std::vector<pcl::PointIndices> &cluster_indices,
                                             int top_n=2);  // 3dbbox估算(单个)
  void estimate_3dbboxs(const cv::Mat &depth,
                        const cv::Mat &intensity,
                        const std::vector<int> &class_ids,
                        const std::vector<std::string> &class_names,
                        const std::vector<int> &bboxes,
                        const std::vector<float> &scores,
                        const std::vector<int> &track_ids,
                        std::vector<Eigen::Matrix<float, 1, 9>> &bbox3ds);  // 3dbbox估算(多个)

private:
  int verbose_;
  Eigen::Matrix3f K_;         // 深度相机内参
  Eigen::Matrix3f K_inv_;     // 深度相机内参逆矩阵
  Eigen::Matrix4f T_l2c_;     // 雷达-相机外参，参考系为相机坐标系，可以近似 pose=(0, 0, 0, pi/2, -pi/2, 0)
  Eigen::Matrix4f T_l2c_inv_; // 雷达-相机外参的逆
  Eigen::Matrix4f T_l2w_;     // 套件位姿(Livox坐标系)齐次矩阵，参考系为世界坐标系
  Eigen::Matrix4f T_l2w_inv_; // 套件位姿的逆

  Params param_;              // 数据处理参数

  cv::FileStorage fs_class_;  // 类别信息配置
};


Estimate3DBBox::Estimate3DBBox(int verbose)
{
  verbose_ = verbose;

  K_.setIdentity();
  K_inv_.setIdentity();
  T_l2c_.setIdentity();
  T_l2c_inv_.setIdentity();
  T_l2w_.setIdentity();
  T_l2w_inv_.setIdentity();

  param_.obj_cloud_pattern.w_points = 0.4;
  param_.obj_cloud_pattern.w_distance = 0.3;
  param_.obj_cloud_pattern.w_matching = 0.3;
  param_.obj_cloud_pattern.gather_coef = 1.0;
  param_.obj_cloud_pattern.pose_type = 0;
}

Estimate3DBBox::~Estimate3DBBox()
{

}

// 设置/获取调试等级
void Estimate3DBBox::set_verbose(int verbose)
{
  verbose_ = verbose;
}
int  Estimate3DBBox::get_verbose()
{
  return verbose_;
}

// 相机内参
void Estimate3DBBox::set_K(Eigen::Matrix3f &K)
{
  K_ = K;
  K_inv_ = K.inverse();
}
const Eigen::Matrix3f &Estimate3DBBox::get_K()
{
  return K_;
}
const Eigen::Matrix3f &Estimate3DBBox::get_K_inv()
{
  return K_inv_;
}

// 雷达-相机外参
void Estimate3DBBox::set_T_l2c(const Eigen::Matrix4f &T)
{
  T_l2c_ = T;
  T_l2c_inv_ = T.inverse();
}
const Eigen::Matrix4f &Estimate3DBBox::get_T_l2c()
{
  return T_l2c_;
}
const Eigen::Matrix4f &Estimate3DBBox::get_T_l2c_inv()
{
  return T_l2c_inv_;
}

// 雷达位姿齐次矩阵, 参考坐标是世界坐标
void Estimate3DBBox::set_T_l2w(const Eigen::Matrix4f &T)
{
  T_l2w_ = T;
  T_l2w_inv_ = T.inverse();
}
const Eigen::Matrix4f &Estimate3DBBox::get_T_l2w()
{
  return T_l2w_;
}
const Eigen::Matrix4f &Estimate3DBBox::get_T_l2w_inv()
{
  return T_l2w_inv_;
}

// 设置强度值滤波参数
void Estimate3DBBox::set_intensity(float thr_val)
{
  param_.intensity.thr_val = thr_val;
}

// 设置地面过滤参数
void Estimate3DBBox::set_filter_ground(float d_threshold, float n_threshold, float percent)
{
  param_.filter_ground.d_threshold = d_threshold;
  param_.filter_ground.n_threshold = n_threshold;
  param_.filter_ground.percent = percent;
}

// 设置欧氏分割参数
void Estimate3DBBox::set_segment_euclidean_cluster(float coeff, int min_size, int max_size)
{
  param_.sec.coeff = coeff;
  param_.sec.min_size = min_size;
  param_.sec.max_size = max_size;
}


/** @brief 加载雷达-相机外参
 * @param in sfile - 外参文件路径
 * @return 是否加载成功
 */
bool Estimate3DBBox::load_extrinsic_l2c(const std::string &sfile)
{
  if (!CommonFuns::load_extrinsic(sfile, T_l2c_)) {
    // 加载外参文件失败，使用 默认位姿(0, 0, 0, M_PI/2, -M_PI/2, 0)
    Eigen::Matrix<float, 1, 6> pose;
    pose << 0, 0, 0, M_PI/2, -M_PI/2, 0;
    T_l2c_ = CommonFuns::pose2T(pose);
  }
  T_l2c_inv_ = T_l2c_.inverse();
  return true;
}


/** @brief 雷达安装位姿，参考坐标系为世界坐标系
 * @param in pose - 套件位姿
 */
bool Estimate3DBBox::load_extrinsic_l2w(const std::string &pose)
{
  if (pose !="" ) {
    T_l2w_ = CommonFuns::pose2T(CommonFuns::pose_from_str(pose));
  }
  T_l2w_inv_ = T_l2w_.inverse();
  return true;
}


/** @brief 加载类别配置文件
 * @param in cfg_file - 配置文件，规范见文件头说明
 * @return 是否加载成功
 */
bool Estimate3DBBox::load_class_info(const std::string &cfg_file)
{
  return fs_class_.open(cfg_file, cv::FileStorage::READ);
}


/** @brief 获取类别信息
 * @param in class_id - 类别ID
 * @param out info    - 类别信息
 * @return 是否获取成功
 */
bool Estimate3DBBox::get_class_info(int class_id, ClassInfo &info)
{
  if (!fs_class_.isOpened()) {
    return false;
  }
  if (fs_class_["classes"].empty()) {
    return false;
  }
  if (fs_class_["classes"][std::to_string(class_id)].empty()) {
    return false;
  }

  info.id = class_id;
  if (!fs_class_["classes"][std::to_string(class_id)]["en_name"].empty()) {
    info.en_name = fs_class_["classes"][std::to_string(class_id)]["en_name"].string();
  }
  if (!fs_class_["classes"][std::to_string(class_id)]["cn_name"].empty()) {
    info.cn_name = fs_class_["classes"][std::to_string(class_id)]["cn_name"].string();
  }
  if (!fs_class_["classes"][std::to_string(class_id)]["size"].empty()) {
    info.size[0] = fs_class_["classes"][std::to_string(class_id)]["size"][0].real();
    info.size[1] = fs_class_["classes"][std::to_string(class_id)]["size"][1].real();
    info.size[2] = fs_class_["classes"][std::to_string(class_id)]["size"][2].real();
  } else {
    info.size[0] = info.size[1] = info.size[2] = 0;
  }
  if (!fs_class_["classes"][std::to_string(class_id)]["conf_th"].empty()) {
    info.conf_th = fs_class_["classes"][std::to_string(class_id)]["iou_th"].real();
  } else {
    info.conf_th = 0.3;
  }
  if (!fs_class_["classes"][std::to_string(class_id)]["iou_th"].empty()) {
    info.iou_th = fs_class_["classes"][std::to_string(class_id)]["iou_th"].real();
  } else {
    info.iou_th = 0.5;
  }
  if (!fs_class_["classes"][std::to_string(class_id)]["icon"].empty()) {
    info.icon = fs_class_["classes"][std::to_string(class_id)]["icon"].string();
  }
  if (!fs_class_["classes"][std::to_string(class_id)]["warn_level"].empty()) {
    info.warn_level = (int)fs_class_["classes"][std::to_string(class_id)]["warn_level"].real();
  } else {
    info.warn_level = 0;
  }

  return true;
}

/** @brief 设置模板匹配选择目标区域参数
 * @param w_matching in - 匹配度权重
 * @param w_distance in - 距离权重
 * @param w_points   in - 点云点数权重
 * @param pose_type  in - 位姿估算方法，0-由PCA估算， 1-由聚合点云边界确定(忽略姿态信息)
 * @param norm_type  in - 正则化方法： 0-最大值比值法2， 1-总和比值法，详细看 obj_cloud_pattern.h
 */
void Estimate3DBBox::set_obj_cloud_pattern_params(
    float w_matching, float w_distance, float w_points, float gather_coef, int pose_type, int norm_type)
{
  param_.obj_cloud_pattern.w_points    = w_points;
  param_.obj_cloud_pattern.w_distance  = w_distance;
  param_.obj_cloud_pattern.w_matching  = w_matching;
  param_.obj_cloud_pattern.gather_coef = gather_coef;
  param_.obj_cloud_pattern.pose_type   = pose_type;
  param_.obj_cloud_pattern.norm_type = norm_type;
}

/** @brief 获取模板匹配选择目标区域参数
 * @param w_matching out - 匹配度权重
 * @param w_distance out - 距离权重
 * @param w_points   out - 点云点数权重
 * @param pose_type  out - 位姿估算方法，0-由PCA估算， 1-由聚合点云边界确定(忽略姿态信息)
 * @param norm_type  out - 正则化方法： 0-最大值比值法2， 1-总和比值法，详细看 obj_cloud_pattern.h
 */
void Estimate3DBBox::get_obj_cloud_pattern_params(
    float &w_matching, float &w_distance, float &w_points, float &gather_coef, int &pose_type, int &norm_type)
{
  w_points    = param_.obj_cloud_pattern.w_points ;
  w_distance  = param_.obj_cloud_pattern.w_distance ;
  w_matching  = param_.obj_cloud_pattern.w_matching ;
  gather_coef = param_.obj_cloud_pattern.gather_coef ;
  pose_type   = param_.obj_cloud_pattern.pose_type;
  norm_type   = param_.obj_cloud_pattern.norm_type;
}

/** @brief 计算欧氏分割邻近点阈值
 *    详细算法描述文档看 2022-05-23工作汇报-黄锦威.odp
 * 场景说明:
 *  1. 欧氏分割效果与邻近点阈值相关，阈值的设置与目标距离相关。阈值大小与目标距离成正相关.
 * 算法模型
 * 已知:
 *    图像宽度 img_w
 *    视场角(水平) fov
 *    光束发散角度(水平) divergence
 *    目标像素宽 obj_w
 * 求：
 *    目标框内雷达扫描点数(水平，理论值) points
 * 解:
 *    points = (obj_w / img_w) * fov / divergence
 *    进一步求解 邻近点阈值
 *
 * @param in img_w - 图像宽度, e.g. 640 pixels
 * @param in fov   - 雷达水平视场角(弧度), Livox Horizon: 81.7度
 * @param in divergence - 光束发散角度(弧度), Livox Horizon: 0.03度
 * @param in obj_r - 目标半径(m)，一般取水平轴的半径,e.g. 人:0.3m
 * @param in obj_w - 目标宽度像素点
 * @param in coeff     - 欧氏分割邻近点阈值系数, 数倍于激光点距，避免扫描点缺失导致搜索间断， e.g. 20
 * @return tole - 欧氏分割邻近点阈值
 */
float Estimate3DBBox::caculate_sec_tole(
    int img_w, float fov, float divergence,
    float obj_r, int obj_w, float coeff)
{
  // angle = (obj_w / img_w) * fov
  // points = angle / divergence
  // point_dist = obj_r / points
  // tole = point_dist * coeff
  if (verbose_>0) {
    printf("[I:%s:%d][caculate_sec_tole]  inputs: img_w=%d, fov=%f, divergence=%f, obj_r=%f, obj_w=%d, coeff=%f\n",
           __FILE__, __LINE__,
           img_w, fov, divergence, obj_r, obj_w, coeff);
    float angle = obj_w*1.0/img_w * fov;
    float points = angle / divergence;
    float point_d = obj_r / points;
    float tole = point_d * coeff;
    printf("    result of caculate ===>\n");
    printf("       angle of obj: %f\n", angle);
    printf("       points o obj: %f\n", points);
    printf("     point_d of obj: %f\n", point_d);
    printf("        tole of obj: %f\n", tole);
  }
  return (obj_r * img_w * divergence * coeff) / (obj_w * fov);
}


/** @brief 提取目标点云
 * 参数解释参考 common.h
 */
void Estimate3DBBox::extract_obj(const cv::Mat &depth, const cv::Mat &intensity, const Eigen::Vector4i &bbox,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr &points)
{
  CommonFuns::extract_obj(depth, intensity, bbox, K_inv_, T_l2c_inv_, T_l2w_, points);
}


/** @brief 地面过滤
 *  参数解释参考 common.h
 */
void Estimate3DBBox::filter_ground(int obj_index, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter)
{
  CommonFuns::filter_ground(obj_index, cloud,
                            param_.filter_ground.d_threshold, param_.filter_ground.n_threshold,
                            param_.filter_ground.percent, param_.filter_ground.min_size,
                            verbose_,
                            cloud_filter);
}


/** @brief 过滤噪声
 */
void Estimate3DBBox::filter_noise(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  if (!cloud || cloud->empty()) {
    printf("[W:%s:%d][filter_noise] cloud is empty!\n", __FILE__, __LINE__);
    return;
  }
}


/** @brief 获取3DBBox
 *  @param cloud_in in 点云
 *  @param indices in 点云索引
 *  @return coeff 3DBBox: [x, y, z, R, P, Y, L, W, H], (x, y, z, R, P, Y)-位姿, (L, W, H)-长宽高
 */
Eigen::Matrix<float, 1, 9> Estimate3DBBox::get_3dbbox(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                                      const pcl::PointIndices &indices, int &num_points, float &distance)
{
  num_points = 0;
  distance = 0;

  if (!cloud || cloud->empty() || cloud->points.size()<=0) {
    printf("[W:%s:%d][get_3dbbox] cloud is empty!\n", __FILE__, __LINE__);
    return Eigen::Matrix<float, 1, 9>().Zero();
  }
  if (indices.indices.size()<=0) {
    printf("[W:%s:%d][get_3dbbox] indices is empty!\n", __FILE__, __LINE__);
    return Eigen::Matrix<float, 1, 9>().Zero();
  }

  Eigen::Matrix<float, 1, 9> bbox3d;

  Eigen::Vector4f minP;
  Eigen::Vector4f maxP;
  pcl::getMinMax3D(*cloud, indices, minP, maxP);

  bbox3d[0] = (minP[0] + maxP[0]) / 2; // x
  bbox3d[1] = (minP[1] + maxP[1]) / 2; // y
  bbox3d[2] = (minP[2] + maxP[2]) / 2; // z
  bbox3d[3] = 0.0;                     // R
  bbox3d[4] = 0.0;                     // P
  bbox3d[5] = 0.0;                     // Y
  bbox3d[6] = maxP[0] - minP[0];       // L
  bbox3d[7] = maxP[1] - minP[1];       // W
  bbox3d[8] = maxP[2] - minP[2];       // H

  num_points = indices.indices.size();
  distance = sqrt(powf(bbox3d[0], 2.0) + powf(bbox3d[1], 2.0) + powf(bbox3d[2], 2.0));

  return bbox3d;
}


/** @brief 3DBBox估算
 */
Eigen::Matrix<float, 1, 9> Estimate3DBBox::estimate_3dbbox(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                           const std::vector<pcl::PointIndices> &cluster_indices,
                                           int top_n)
{
  if (!cloud || cloud->empty() || cloud->points.size()<=0) {
    printf("[W:%s:%d][estimate_3dbbox] cloud is empty!\n", __FILE__, __LINE__);
    return Eigen::Matrix<float, 1, 9>().Zero();
  }
  if (cluster_indices.size()<=0) {
    printf("[W:%s:%d][estimate_3dbbox] cluster_indices is empty!\n", __FILE__, __LINE__);
    return Eigen::Matrix<float, 1, 9>().Zero();
  }

  std::vector<Eigen::Matrix<float, 1, 9>> bbox3ds; // 先把聚类序列中各个的dbbbox储存
  std::vector<int> bbox3ds_points;                 // 记录点数量
  std::vector<float> bbox3ds_dis;                  // 记录中心距离
  int max_points = 0;                              // 记录最大点数量
  float max_distance = 0;                          // 记录最大中心距离

  for(unsigned i=0; i<cluster_indices.size() && (top_n<=0 || (int)i < top_n); i++) {
    int num_points = 0;
    float distance = 0;
    bbox3ds.push_back(get_3dbbox(cloud, cluster_indices[i], num_points, distance));
    bbox3ds_points.push_back(num_points);
    bbox3ds_dis.push_back(distance);
    if (num_points > max_points) {
      max_points = num_points;
    }
    if (distance > max_distance) {
      max_distance = distance;
    }
  }

  // 挑选其中一个3dbbox返回
  //float w_points = 0.5;   // - 点数量权重
  //float w_distance = 0.5; // - 距离权重
  return bbox3ds[0];
}

/** @brief 3DBBox估算
 * @param in depth - 深度图: H x W, float32
 * @param in intensity - 强度值图: H x W, float32
 * @param in class_ids - 类别ID向量
 * @param in class_names - 类别名称向量
 * @param in bboxes -  2DBBox, [x1_1, y1_1, x1_2, y1_2, ...]
 * @param in scores - 置信度向量
 * @param in track_ids - 目标跟踪ID向量
 * @param out bbox3ds - 返回3DBBox估算值向量: [[x, y, z, R, P, Y, L, W, H],...]
 */
void Estimate3DBBox::estimate_3dbboxs(
    const cv::Mat &depth,
    const cv::Mat &intensity,
    const std::vector<int> &class_ids,
    const std::vector<std::string> &class_names,
    const std::vector<int> &bboxes,
    const std::vector<float> &scores,
    const std::vector<int> &track_ids,
    std::vector<Eigen::Matrix<float, 1, 9>> &bbox3ds)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());

  // 目标个数
  int n_objs = (int)class_ids.size();
  if(verbose_>0) {
    printf("[I:%s:%d][estimate_3dbboxs] total %d objects\n", __FILE__, __LINE__, n_objs);
  }

  //  模板匹配选取目标区域
  float w_matching  = param_.obj_cloud_pattern.w_matching;
  float w_distance  = param_.obj_cloud_pattern.w_distance;
  float w_points    = param_.obj_cloud_pattern.w_points;
  float gather_coef = param_.obj_cloud_pattern.gather_coef;
  int pose_type     = param_.obj_cloud_pattern.pose_type;
  int norm_type     = param_.obj_cloud_pattern.norm_type;
  ObjCloudPattern obj;
  obj.setVerbose(verbose_);
  obj.setWeights(w_matching, w_distance, w_points);
  obj.setGatherCoeff(gather_coef);
  obj.setPoseType(pose_type);
  obj.setNormType(norm_type);

  // 遍历bbox
  for(int i=0; i < n_objs; i++)
  {
      if (verbose_>0) {
        printf("========================= obj : %d ==================================\n", i);
        printf("[I:%s:%d][estimate_3dbboxs] process the %d object, name='%s', class_id=%d\n",
           __FILE__, __LINE__, i, class_names[i].c_str(), class_ids[i]);
      }

      // 提取目标bbox: [x1, y1, x2, y2]
      // ==============================
      Eigen::Vector4i bbox;
      for (int j=0; j<4; j++) {bbox[j] = bboxes[i*4+j];}

      // 提取目标点云区域
      // ==============
      pcl::PointCloud<pcl::PointXYZI>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      extract_obj(depth, intensity, bbox, obj_cloud);
      if (obj_cloud->points.size() <=0) {
        bbox3ds.push_back(Eigen::Matrix<float, 1, 9>::Zero());
        continue;
      }

      if (verbose_>1)
      {
        Eigen::Vector4f min_val;
        Eigen::Vector4f max_val;
        Eigen::Vector4f mean_val;
        CommonFuns::points_stat(obj_cloud, min_val, max_val, mean_val);
        printf("[I:%s:%d][estimate_3dbboxs] extract object\n", __FILE__, __LINE__);
        printf("[I:%s:%d][estimate_3dbboxs] points stat: %ld points \n"
               "    bbox(%d, %d, %d, %d)\n"
               "     min(%f, %f, %f, %f)\n"
               "     max(%f, %f, %f, %f)\n"
               "    mean(%f, %f, %f, %f)\n", __FILE__, __LINE__,
               obj_cloud->points.size(),
               bbox(0), bbox(1), bbox(2), bbox(3),
               min_val(0), min_val(1), min_val(2), min_val(3),
               max_val(0), max_val(1), max_val(2), max_val(3),
               mean_val(0), mean_val(1), mean_val(2), mean_val(3));
        char pcd_file[255] = {};
        sprintf(pcd_file, "/tmp/obj_cloud_%d.pcd", i);
        pcl::io::savePCDFile(pcd_file, *obj_cloud);
        printf("    save to '%s'\n", pcd_file);
      }

      // 过滤地面
      // =======
      cloud_temp->clear();
      filter_ground(i, obj_cloud, cloud_temp);
      if (cloud_temp->points.size()>0) {
        *obj_cloud = *cloud_temp ;
      } else {
        printf("[W:%s:%d]Ignore filter_ground because return empty result!!\n", __FILE__, __LINE__);
      }
      if(obj_cloud->points.size() <= 0) {
        bbox3ds.push_back(Eigen::Matrix<float, 1, 9>::Zero());
        continue;
      }

      if (verbose_>1) {
        Eigen::Vector4f min_val;
        Eigen::Vector4f max_val;
        Eigen::Vector4f mean_val;
        CommonFuns::points_stat(obj_cloud, min_val, max_val, mean_val);
        printf("[I:%s:%d][estimate_3dbboxs] After filter ground, points stat: %ld points \n"
               "    bbox(%d, %d, %d, %d)\n"
               "     min(%f, %f, %f, %f)\n"
               "     max(%f, %f, %f, %f)\n"
               "    mean(%f, %f, %f, %f)\n", __FILE__, __LINE__,
               obj_cloud->points.size(),
               bbox(0), bbox(1), bbox(2), bbox(3),
               min_val(0), min_val(1), min_val(2), min_val(3),
               max_val(0), max_val(1), max_val(2), max_val(3),
               mean_val(0), mean_val(1), mean_val(2), mean_val(3));
        char pcd_file[255] = {};
        sprintf(pcd_file, "/tmp/obj_cloud_filter_ground_%d.pcd", i);
        pcl::io::savePCDFile(pcd_file, *obj_cloud);
        printf("    save to '%s'\n", pcd_file);
      }

      // 噪声过滤
      // =======
      filter_noise(obj_cloud);
      if(obj_cloud->points.size() <= 0) {
        bbox3ds.push_back(Eigen::Matrix<float, 1, 9>::Zero());
        continue;
      }
      if (verbose_>1) {
        Eigen::Vector4f min_val;
        Eigen::Vector4f max_val;
        Eigen::Vector4f mean_val;
        CommonFuns::points_stat(obj_cloud, min_val, max_val, mean_val);
        printf("[I:%s:%d][estimate_3dbboxs] After filter noise, points stat: %ld points \n"
               "    bbox(%d, %d, %d, %d)\n"
               "     min(%f, %f, %f, %f)\n"
               "     max(%f, %f, %f, %f)\n"
               "    mean(%f, %f, %f, %f)\n", __FILE__, __LINE__,
               obj_cloud->points.size(),
               bbox(0), bbox(1), bbox(2), bbox(3),
               min_val(0), min_val(1), min_val(2), min_val(3),
               max_val(0), max_val(1), max_val(2), max_val(3),
               mean_val(0), mean_val(1), mean_val(2), mean_val(3));
        char pcd_file[255] = {};
        sprintf(pcd_file, "/tmp/obj_cloud_filter_noise_%d.pcd", i);
        pcl::io::savePCDFile(pcd_file, *obj_cloud);
        printf("    save to '%s'\n", pcd_file);
      }

      // 欧氏分割
      // =======
      std::vector<pcl::PointIndices> cluster_indices;
      float tole = 0.6 - 0.005 * (bbox(2)-bbox(0));
      if (tole < 0.05) {
        tole = 0.05;
      }
      float r = 1.5;
      float L = 1.5;
      float W = 1.5;
      float H = 1.5;

      if (class_ids[i] == 0) {
        // 人
        r = 0.5;
        L = 0.5;
        W = 0.5;
        H = 1.7;
        // tole = caculate_sec_tole(640, 81.7*3.1415926/180, 0.03*3.1415926/180,  0.5, bbox(2)-bbox(0), param_.sec.coeff);
        // obj.setClassInfo(0.5, 0.5, 1.7);
      } else if (class_ids[i]==2){
        // 车
        r = 6.0;
        L = 6.0;
        W = 1.8;
        H = 1.4;
        // tole = caculate_sec_tole(640, 81.7*3.1415926/180, 0.03*3.1415926/180,  6.0, bbox(2)-bbox(0), param_.sec.coeff);
        // obj.setClassInfo(6.0, 1.8, 1.4);
      } else if (class_ids[i]==7) {
        // 卡车
        r = 12.0;
        L = 12.0;
        W = 3.0;
        H = 3.5;
        // tole = caculate_sec_tole(640, 81.7*3.1415926/180, 0.03*3.1415926/180,  12.0, bbox(2)-bbox(0), param_.sec.coeff);
        // obj.setClassInfo(12.0, 3.0, 3.5);
      } else if (class_ids[i]==10) {
        // 消防栓
        r = 0.5;
        L = 0.15;
        W = 0.15;
        H = 0.8;
      }
      tole = caculate_sec_tole(640, 81.7*3.1415926/180, 0.03*3.1415926/180,  r, bbox(2)-bbox(0), param_.sec.coeff);
      obj.setClassInfo(L, W, H);
      CommonFuns::segment_euclidean_cluster(obj_cloud, cluster_indices, tole, param_.sec.min_size, param_.sec.max_size);
      if (verbose_>1) {
        printf("[I:%s:%d][estimate_3dbboxs][segment_euclidean_cluster] "
               "%ld clusters, tole=%f, min_size=%d, max_size=%d, bbox=(%d,%d,%d,%d)\n",
               __FILE__, __LINE__,
               cluster_indices.size(), tole, param_.sec.min_size, param_.sec.max_size, bbox(0), bbox(1), bbox(2), bbox(3));
        for (unsigned int j=0; j<cluster_indices.size(); j++) {
          printf("[I:%s:%d][estimate_3dbboxs][segment_euclidean_cluster][index %d] %ld points in cluster\n",
                 __FILE__, __LINE__, j, cluster_indices[j].indices.size());
          if (verbose_>2) {
            char pcd_file[255] = {};
            sprintf(pcd_file, "/tmp/obj_cloud_seg_%d_%d.pcd", i, j);
            pcl::PointCloud<pcl::PointXYZI>::Ptr obj_cluster(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointIndices::Ptr indices(new pcl::PointIndices(cluster_indices[j]));
            CommonFuns::extract_points(obj_cloud, indices, obj_cluster);
            pcl::io::savePCDFile(pcd_file, *obj_cluster);
            printf("    save to '%s'\n", pcd_file);
          }
        }
      }

      // 3dbbox估算: [x, y, z, R, P, Y, L, W, H]
      // ======================================
      if (param_.obj_cloud_pattern.pose_type==2) {
        Eigen::Matrix<float, 1, 9> bbox3d = estimate_3dbbox(obj_cloud, cluster_indices, 2);
        bbox3ds.push_back(bbox3d);
      } else {
        if (cluster_indices.size()>0) {
          // 有聚类结果: 通过模板匹配选择最佳聚类子列
          Eigen::Matrix<float, 1, 9> bbox3d;
          obj.apply(i, obj_cloud, cluster_indices, bbox3d);
          bbox3ds.push_back(bbox3d);
        } else {
          // 无聚类结果: 重新设置欧氏分割阈值，选择首个聚类结果返回
          CommonFuns::segment_euclidean_cluster(obj_cloud, cluster_indices, r, 1, param_.sec.max_size);
          Eigen::Matrix<float, 1, 9> bbox3d = estimate_3dbbox(obj_cloud, cluster_indices, 2);
          bbox3ds.push_back(bbox3d);
        }
      }
      if (verbose_>0) {
        printf("\n");
      }
  }
}

#endif // ESTIMATE_3DBBOX_H
