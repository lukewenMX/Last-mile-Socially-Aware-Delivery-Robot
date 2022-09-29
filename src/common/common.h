/** @brief 通用函数集
 *
 * version: 0.0.2
 *
 * 更新记录
 * =======
 * - 添加强度值支持
 * version: 0.0.3
 *  添加 save_data, read_data, points_stat接口
 *  重载 depth2cloud
 */
#ifndef COMMON_H
#define COMMON_H

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

class CommonFuns
{
public:
  // 去掉字符串左边空格: inplace
  static std::string& ltrim(std::string &ss);
  // 去掉字符串右边空格: inplace
  static std::string& rtrim(std::string &ss);
  // 去掉字符串两边空格: inplace
  static std::string& trim(std::string &st);
  // 字符串分割
  static std::vector<std::string> string_split(const  std::string& s, const std::string& delim);
  // 判断文件是否存在
  static bool IsFileExist(const char *file_name);
  // 变换矩阵转位姿
  static Eigen::Matrix4f pose2T(const Eigen::Matrix<float, 1, 6> &pose);
  // 位姿转变换矩阵
  static Eigen::Matrix<float, 1, 6> T2pose(const Eigen::Matrix4f &T);
  // 从字符串解析位姿
  static Eigen::Matrix<float, 1, 6> pose_from_str(const std::string &str);
  // 加载内参
  static bool load_intrinsic(const std::string &sfile, Eigen::Matrix3f &K, Eigen::Matrix<float, 1, 5> &D);
  // 加载单应性矩阵
  static bool load_homograph(const std::string &sfile, Eigen::Matrix3f &H);
  // 加载外参
  static bool load_extrinsic(const std::string &sfile, Eigen::Matrix4f &T);
  // 获取颜色值
  static unsigned get_color(int index);

  // uvw => (x, y, z)
  static void uvw2xyz(int u, int v, float w, float &x, float &y, float &z,
                     const Eigen::Matrix3f &K_inv, const Eigen::Matrix4f &T_l2c_inv, const Eigen::Matrix4f &T_l2w);
  // 点云深度图投影: cloud[PointXYZI] => (depth, intensity)
  static void cloud2depth(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                          cv::Mat &depth, cv::Mat &intensity /* out */,
                          const Eigen::Matrix3f &K, const Eigen::Matrix<float, 3, 4> &Ka, const Eigen::Matrix4f &T,
                          float min_val = 1.0,
                          float max_val = 15.0);
  // 深度图转点云: depth => cloud[PointXYZ]
  static void depth2cloud(const cv::Mat &depth,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud /* out */,
                          const Eigen::Matrix3f &K_inv, const Eigen::Matrix4f &T_l2c_inv, const Eigen::Matrix4f &T_l2w);
  // 深度图转点云: (depth, intensity) => cloud[PointXYZI]
  static void depth2cloud(const cv::Mat &depth, const cv::Mat &intensity,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud /* out */,
                          const Eigen::Matrix3f &K_inv, const Eigen::Matrix4f &T_l2c_inv, const Eigen::Matrix4f &T_l2w);
  // 深度图转点云: (rgb, depth) => cloud[PointXYZRGB]
  static void depth2cloud(const cv::Mat &rgb, const cv::Mat &depth, const std::vector<int> &bboxes,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud /* out */,
                          const Eigen::Matrix3f &K_inv, const Eigen::Matrix4f &T_l2c_inv, const Eigen::Matrix4f &T_l2w);
  // 彩色图投影点云: (image, cloud[PointXYZI]) => cloud[PointXYZRGB]
  static void rgb2cloud(const cv::Mat &image, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloudPtr,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colorCloud /* out */,
                        const cv::Mat &K, const cv::Mat & D, const cv::Mat &T_l2c);
  // 彩色图投影点云: (image, cloud[PointXYZI]) => cloud[PointXYZRGB]
  static void rgb2cloud(const cv::Mat &image, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colorCloud /* out */,
                        const Eigen::Matrix3f &K, const Eigen::Matrix<float, 1, 5> &D,
                        const Eigen::Matrix4f &T_l2c, const Eigen::Matrix4f &T_l2w);
  // 点云着色
  static void cloud_mask_color(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in,
                               const std::vector<pcl::PointIndices> &cluster_indices,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out /* out */);
  // 点云着色
  static void cloud_mask_color(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in,
                               int color,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out /* out */);

  // 保存测试数据
  static void save_data(
      const std::string &spath, int frame_index,
      const cv::Mat &img,
      const cv::Mat &depth,
      const cv::Mat &intensity,
      const std::vector<int> &class_ids,
      const std::vector<std::string> &class_names,
      const std::vector<int> &bboxes,
      const std::vector<float> &scores,
      const std::vector<int> &track_ids);
  // 加载测试数据: 所有数据（含检测结果）
  static void read_data(
      const std::string &spath,
      int frame_index,
      cv::Mat &img,
      cv::Mat &depth,
      cv::Mat &intensity,
      std::vector<int> &class_ids,
      std::vector<std::string> &class_names,
      std::vector<int> &bboxes,
      std::vector<float> &scores,
      std::vector<int> &track_ids);
  // 加载测试数据: 不含检测结果
  static void read_data(
      const std::string &spath,
      int frame_index,
      cv::Mat &img,
      cv::Mat &depth,
      cv::Mat &intensity);
  // 点云统计
  static void points_stat(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                          Eigen::Vector4f &min_val,
                          Eigen::Vector4f &max_val,
                          Eigen::Vector4f &mean_val);
  // 提取目标点云
  static void extract_obj(const cv::Mat &depth, const cv::Mat &intensity, const Eigen::Vector4i &bbox,
                          const Eigen::Matrix3f &K_inv, const Eigen::Matrix4f &T_l2c_inv, const Eigen::Matrix4f &T_l2w,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr &points);
  // 提取点云
  static void extract_points(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in,
                             const pcl::PointIndices::ConstPtr indices,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out);
  // 地面过滤
  static void filter_ground(int obj_index, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                            float d_threshold, float n_threshold, float percent, int min_size,
                            int verbose,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_filter);
  // 欧氏分割
  static void segment_euclidean_cluster(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                                        std::vector<pcl::PointIndices> &cluster_indices,
                                        float tole, int min_size, int max_size);
};



// 去掉字符串左边空格: inplace
std::string& CommonFuns::ltrim(std::string &ss)
{
    std::string::iterator p=std::find_if(ss.begin(), ss.end(), std::not1(std::ptr_fun(isspace)));
    ss.erase(ss.begin(),p);
    return ss;
}
// 去掉字符串右边空格: inplace
std::string& CommonFuns::rtrim(std::string &ss)
{
    std::string::reverse_iterator  p=std::find_if(ss.rbegin(), ss.rend(), std::not1(std::ptr_fun(isspace)));
    ss.erase(p.base(),ss.end());
    return ss;
}
// 去掉字符串两边空格: inplace
std::string& CommonFuns::trim(std::string &st)
{
    ltrim(rtrim(st));
    return st;
}

/** @brief 字符串分割
 * @param in s - 被分割字符串
 * @param in delim - 分割符
 * @return 分割向量结果
 */
std::vector<std::string> CommonFuns::string_split(const  std::string& s, const std::string& delim)
{
    std::vector<std::string> elems;
    size_t pos = 0;
    size_t len = s.length();
    size_t delim_len = delim.length();
    if (delim_len == 0) return elems;
    while (pos < len)
    {
        int find_pos = s.find(delim, pos);
        if (find_pos < 0)
        {
            elems.push_back(s.substr(pos, len - pos));
            break;
        }
        elems.push_back(s.substr(pos, find_pos - pos));
        pos = find_pos + delim_len;
    }
    return elems;
}


/** @brief 判断文件是否存在
 */
bool CommonFuns::IsFileExist(const char *file_name)
{
  try
  {
    FILE *fp=fopen(file_name, "rb");
    if(fp!=NULL)
    {
      fclose(fp);
      return true;
    }
    else
    {
      return false;
    }
  }catch(...)
  {
    return false;
  }
}


/** @brief pose2T
 *  位姿转的齐次矩阵表达
 *    T <- Trans(x, y, z) * RotZ(Y) * RotY(P) * RotX(R)
 * @param in pose - 位姿(x, y, z, R, P, Y)
 * @return T - 4x4变换矩阵
 */
Eigen::Matrix4f CommonFuns::pose2T(const Eigen::Matrix<float, 1, 6> &pose)
{
  Eigen::Isometry3f T;
  Eigen::Vector3f   vec(pose[0], pose[1], pose[2]);         // 偏移向量
  Eigen::AngleAxisf rx(pose[3], Eigen::Vector3f::UnitX());  // R
  Eigen::AngleAxisf ry(pose[4], Eigen::Vector3f::UnitY());  // P
  Eigen::AngleAxisf rz(pose[5], Eigen::Vector3f::UnitZ());  // Y
  T.setIdentity();
  // 获取变换矩阵: T = trans(x, y, z) * RotZ（Y） * RotY(P) * RotX(R)
  //                 <=>
  //                 trans.translate(vec) -> trans.rotate(rz) -> trans.rotate(ry) -> trans.rotate(rx)
  T.translate(vec);
  T.rotate(rz);
  T.rotate(ry);
  T.rotate(rx);

  return T.matrix();
}


/** @brief 变换矩阵转位姿
 * @param in T - in 变换矩阵: 4x4
 * @return pose - 位姿(x, y, z, R, P, Y)
 */
Eigen::Matrix<float, 1, 6> CommonFuns::T2pose(const Eigen::Matrix4f &T)
{
  Eigen::Matrix<float, 1, 6> pose;
  Eigen::Isometry3f affine(T);
  Eigen::Matrix3f mat_rotation1, mat_scaling1;
  affine.computeScalingRotation(&mat_scaling1, &mat_rotation1);      // Isometry3f => mat_scaling, mat_rotation
  // (2, 1, 0) <==> RotZ(Y) * RotY(P) * RotX(R)
  Eigen::Vector3f ea1 = mat_rotation1.matrix().eulerAngles(2, 1, 0); // Matrix3f => eulerAngles: 0-rx, 1-ry, 2-rz
  pose[0] = affine.matrix()(0, 3) ;
  pose[1] = affine.matrix()(1, 3) ;
  pose[2] = affine.matrix()(2, 3) ;
  pose[3] = ea1[2] ;
  pose[4] = ea1[1] ;
  pose[5] = ea1[0] ;
  return pose;
}

/** @brief 从字符串解析位姿
 *  @param in str - 位姿， "x,y,z,R,P,Y", 中间不要有空格
 */
Eigen::Matrix<float, 1, 6> CommonFuns::pose_from_str(const std::string &str)
{
  std::vector<std::string> vec = string_split(str, ",");
  Eigen::Matrix<float, 1, 6> pose;
  pose.Zero();

  for (unsigned int i=0; i<vec.size() && i<6; i++) {
    pose[i] = atof(vec[i].c_str());
  }
  return pose;
}

/** @brief 加载内参
 * @param in sfile - 内参文件, 严格按照格式存储
 *    intrinsic
 *    421.427444 0.000000 329.042732
 *    0.000000 420.084118 245.507556
 *    0.000000 0.000000 1.000000
 *
 *    distorsion
 *    0.181593 -0.278716 0.004445 -0.000982 0.140287
 *
 *    rms_err
 *    0.070891
 * @param out K - 内参矩阵
 * @param out D - 畸变参数
 * @return bool 是否加载成功
 */
bool CommonFuns::load_intrinsic(const std::string &sfile, Eigen::Matrix3f &K, Eigen::Matrix<float, 1, 5> &D)
{
  bool ret = false;

  std::cout << "[CommonFuns::load_intrinsic] sfile: " << sfile << std::endl;

  K.setIdentity();
  D.setZero();

  std::ifstream infile;
  infile.open(sfile);
  if (infile.is_open()) {
    std::string line;
    std::getline(infile, line); // 跳过 instrinsic
    std::cout << "line: " << line << std::endl;
    // K: 3x3
    for(unsigned i = 0; i<3; i++) {
      std::getline(infile, line);
      std::cout << "line: " << line << std::endl;
      std::stringstream vals(line);
      float k0, k1, k2;
      vals >> k0 >> k1 >> k2;
      K(i, 0) = k0;
      K(i, 1) = k1;
      K(i, 2) = k2;
    }
    std::getline(infile, line); // 跳过 空行
    std::cout << "line: " << line << std::endl;
    std::getline(infile, line); // 跳过 distorsion
    std::cout << "line: " << line << std::endl;

    // D: 1x5
    std::getline(infile, line);
    std::cout << "line: " << line << std::endl;
    std::stringstream vals(line);
    float d0, d1, d2, d3, d4;
    vals >> d0 >> d1 >> d2 >> d3 >> d4;
    D[0] = d0; D[1] = d1; D[2] = d2; D[3] = d3; D[4] = d4;

    ret = true;
  } else {
    printf("[E:%s:%d][load_intrinsic] %s not exists!\n", __FILE__, __LINE__, sfile.c_str());
  }

  return ret;
}


/** @brief 加载单应性矩阵
 * @param in sfile - 热感-彩色相机单应性矩阵文件
 *    homography_thermal2rgb:
 *    0.879288 -0.027530 29.212638
 *    0.016462 0.874225 13.594919
 *    -0.000001 -0.000025 1.000000
 * @param out H - 单应性矩阵, 3x3
 * @return bool 是否加载成功
 */
bool CommonFuns::load_homograph(const std::string &sfile, Eigen::Matrix3f &H)
{
  bool ret = false;
  std::cout << "[CommonFuns::load_homograph] sfile: " << sfile << std::endl;
  std::ifstream infile;
  infile.open(sfile);
  if (infile.is_open()) {
    std::string line;
    std::getline(infile, line); // 跳过 homography_thermal2rgb
    std::cout << "line: " << line << std::endl;
    // H: 3x3
    for(unsigned i = 0; i<3; i++) {
      std::getline(infile, line);
      std::cout << "line: " << line << std::endl;
      std::stringstream vals(line);
      float h1, h2, h3;
      vals >> h1 >> h2 >> h3;
      H(i, 0) = h1;
      H(i, 1) = h2;
      H(i, 2) = h3;
    }
    ret = true;
  } else {
    printf("[E:%s:%d][load_extrinsic] %s not exists!\n", __FILE__, __LINE__, sfile.c_str());
  }

  return ret;
}


/** @brief 加载外参
 * @param in sfile - 外参文件； 非文件格式，尝试从位姿解析pose:(x, y, z, R, P, Y)
 *    Tr_l2c_min2d:
 *    0.01315800 -0.99964935 -0.02294985 -0.06380704
 *    0.00416756  0.02300716 -0.99972594  0.03727373
 *    0.99990404  0.01305806  0.00446950 -0.04695774
 *    0.00000000  0.00000000  0.00000000  1.00000000
 *
 *    rmse_2d_reproj_wt_centroid_min2d = 294.30795199, 209.44909060, 361.22858713
 * @param out T - 变换矩阵
 * @return bool 是否成功
 */
bool CommonFuns::load_extrinsic(const std::string &sfile, Eigen::Matrix4f &T)
{
  bool ret = false;
  std::cout << "[CommonFuns::load_extrinsic] sfile: " << sfile << std::endl;
  std::ifstream infile;
  infile.open(sfile);
  if (infile.is_open()) {
    // 从文件加载
    std::string line;
    std::getline(infile, line); // 跳过 Tr_l2c_min2d
    std::cout << "line: " << line << std::endl;
    // T: 4x4
    for(unsigned i = 0; i<4; i++) {
      std::getline(infile, line);
      std::cout << "line: " << line << std::endl;
      std::stringstream vals(line);
      float v1, v2, v3, v4;
      vals >> v1 >> v2 >> v3 >> v4;
      T(i, 0) = v1;
      T(i, 1) = v2;
      T(i, 2) = v3;
      T(i, 3) = v4;
    }
    std::cout << T << std::endl;
    ret = true;
  } else {
    if (sfile.substr(sfile.length()-4) != ".txt") {
      // 尝试从位姿解析
      printf("[E:%s:%d][load_extrinsic] %s not exists, try to analysis by pose:[x,y,z,R,P,Y]!\n",
             __FILE__, __LINE__, sfile.c_str());
      const char *str=sfile.c_str();
      int n_split = 0;
      while(char ch=*str++) {
        if (ch==',') {
          n_split ++;
        }
      }
      if (n_split==3) {
        Eigen::Matrix<float, 1, 6> pose = pose_from_str(sfile);
        T = pose2T(pose);
        ret = true;
      } else {
        printf("[E:%s:%d][load_extrinsic] pose: %s invalid formate!\n", __FILE__, __LINE__, sfile.c_str());
      }
    }
    printf("[E:%s:%d][load_extrinsic] %s not exists!\n", __FILE__, __LINE__, sfile.c_str());
  }

  return ret;
}


/** @brief 点云深度图投影
 *  点云 => 深度图， 强度值图
 *                    x T: l2c            x Ka: [K, 0]
 *  雷达坐标系(livox) ----------> 相机坐标系 -------------> 像素坐标系
 *  ===>
 *  pointsUV = Ka * T * pointsL
 *
 * @param in cloud         - 输入点云数据[雷达坐标系]
 * @param in out depth     - 输出深度图, 已定义好大小, CV_32F类型，初始化为0
 * @param in out intensity - 输出强度值图, 大小与depth一致, CV_32F类型，初始化为0
 * @param in K             - 相机内参, 3x3
 * @param in Ka            - 相机内参增广矩阵, 3x4, 最右一列为0
 * @param in T             - 雷达-相机外参(l2c), 4x4
 */
void CommonFuns::cloud2depth(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, cv::Mat &depth, cv::Mat &intensity,
    const Eigen::Matrix3f &K, const Eigen::Matrix<float, 3, 4> &Ka, const Eigen::Matrix4f &T,
    float min_val, float max_val)
{
  if (!cloud || cloud->points.size() <=0) {
    printf("[E:%s:%d][CommonFuns::cloud2depth] cloud is empty!\n", __FILE__, __LINE__);
    return;
  }
  if (depth.empty() || intensity.empty()) {
    printf("[E:%s:%d][CommonFuns::cloud2depth] depth or intensity is empty!\n", __FILE__, __LINE__);
    return;
  }
  if (depth.size != intensity.size) {
    printf("[E:%s:%d][CommonFuns::cloud2depth] depth and intensity is not same size!\n", __FILE__, __LINE__);
    return;
  }

  // depth, intensity 清零
  int nPoints = cloud->points.size();

  // 提取点云齐次坐标: [4 x points]
  Eigen::Matrix<float, 4, Eigen::Dynamic> pointsL;
  pointsL.resize(4, cloud->points.size());
  for (unsigned  int c = 0; c < cloud->points.size(); c++) {
    pointsL(0, c) = cloud->points[c].x;
    pointsL(1, c) = cloud->points[c].y;
    pointsL(2, c) = cloud->points[c].z;
    pointsL(3, c) = 1.0;
  }

  // 2D投影: [3 x points]
  Eigen::Matrix<float, 3, Eigen::Dynamic> pointsUV;
  pointsUV.resize(3, nPoints);
  pointsUV = Ka * T * pointsL;

  // 生成深度图,[480x640]
  for (int c = 0; c < pointsUV.cols(); c++) {
    float scale = pointsUV(2, c);
    int u = (int)(pointsUV(0, c) / scale);
    int v = (int)(pointsUV(1, c) / scale);
    //printf("c=%d,u=%d,v=%d,rows=%d,cols=%d,scale=%f,min_val=%f,max_val=%f\n",
    //        c,   u,   v,   depth.rows, depth.cols, scale, min_val, max_val);
    if (u>=0 && u < depth.cols  && v>=0 && v < depth.rows && scale > min_val && scale < max_val) {
      depth.at<float>(v, u) = scale;                          // 深度值
      intensity.at<float>(v, u) = cloud->points[c].intensity; // 强度值
    }
  }
}

// uvw => (x, y, z)
/** @brief 深度图坐标点(u, v, w) 转换为 三维坐标点(x, y, z)
 *  轻量级的 cloud2depth, 计算方法参考深度图转点云
 * @param u, v, w in    - 深度图坐标点，(u, v)坐标， w-深度值
 * @param x, y, z out   - 三维坐标点
 * @param K_inv in      - 相机内参的逆
 * @param T_l2c_inv in  - 雷达-相机外参的逆
 * @param T_l2w in      - 套件安装位姿
 */
void CommonFuns::uvw2xyz(int u, int v, float w, float &x, float &y, float &z,
                   const Eigen::Matrix3f &K_inv, const Eigen::Matrix4f &T_l2c_inv, const Eigen::Matrix4f &T_l2w)
{
  x = 0; y = 0; z = 0;

  // 提取UV坐标点向量: [3xN]
  Eigen::Matrix<float, 3, 1> pointsUV;
  pointsUV(0, 0) = u * w;
  pointsUV(1, 0) = v * w;
  pointsUV(2, 0) = w;

  // UV坐标系转换相机坐标系: [3xN]
  Eigen::Matrix<float, 4, 1> pointsC;
  pointsC.block(0, 0, 3, 1) = K_inv * pointsUV;   // [3x3] * [3x1] => [3x1]
  pointsC(3, 0) = 1;

  // 相机坐标系转换为雷达坐标系: [4xN]
  Eigen::Matrix<float, 4, 1> pointsL;
  pointsL = T_l2c_inv * pointsC;  // [4x4] * [4xN] => [4xN]

  // 雷达坐标系转换为世界坐标系: [4xN]
  Eigen::Matrix<float, 4, 1> pointsW;
  pointsW = T_l2w * pointsL;

  // 返回三维坐标点
  x = pointsW(0, 0);
  y = pointsW(1, 0);
  z = pointsW(2, 0);
}


/** @brief 深度图转点云
 *  深度图 => 点云(PointXYZ)
 *                      x T_l2w                    x T_l2c              x Ka: [K, 0]
 *  世界坐标系(world) <----------- 雷达坐标系(livox) ----------> 相机坐标系 --------------> 像素坐标系
 *  ===>
 *                                |<- pointsC  ->|
 *                    |<- pointsL -------------->|
 *            |<- pointsW ---------------------->|
 *  pointsW = T_l2w * T_l2c_inv * K_inv * pointsUV
 *
 * @param in depth      - 深度图, CV_32F
 * @param out cloud     - 点云: PointXYZ
 * @param in K_inv      - 相机内参的逆： 3x3
 * @param in T_l2c_inv  - 雷达->相机的标定外参的逆: 4x4
 * @param in T_l2w      - 雷达安装位姿: 4x4
 */
void CommonFuns::depth2cloud(
    const cv::Mat &depth, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const Eigen::Matrix3f &K_inv, const Eigen::Matrix4f &T_l2c_inv, const Eigen::Matrix4f &T_l2w)
{
  if (depth.empty()) {
    printf("[E:%s:%d][CommonFuns::cloud2depth] depth is empty!\n", __FILE__, __LINE__);
    return;
  }
  if (!cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>());
    cloud = temp;
  }

  // 提取UV坐标点向量: [3xN]
  Eigen::Matrix<float, 3, Eigen::Dynamic> pointsUV;
  int n_points = cv::countNonZero(depth);
  pointsUV.resize(3, n_points);
  int point_index = 0;
  for (int u = 0; u < depth.cols; u++) {
    for (int v = 0; v <  depth.rows; v++) {
      float depth_val = depth.at<float>(v, u);
      if (u >=0 && u < depth.cols && v >=0 && v <depth.rows && depth_val>0 && point_index<n_points) {
        pointsUV(0, point_index) = u * depth_val;
        pointsUV(1, point_index) = v * depth_val;
        pointsUV(2, point_index) = depth_val;
        point_index++;
      }
    }
  }

  // UV坐标系转换相机坐标系: [3xN]
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> pointsC;
  pointsC.resize(3, n_points);  // (3xN)
  pointsC = K_inv * pointsUV;   // [3x3] * [3xN] => [3xN]

  // 相机坐标系点向量转换的齐坐标: [3xN] => [4xN]
  pointsC.conservativeResize(pointsC.rows()+1, pointsC.cols()); // [3xN] => [4xN], 添加一行 => 齐次坐标
  pointsC.row(pointsC.rows() - 1) = Eigen::MatrixXf::Ones(1, pointsC.cols());

  // 相机坐标系转换为雷达坐标系: [4xN]
  Eigen::Matrix<float, 4, Eigen::Dynamic> pointsL;
  pointsL = T_l2c_inv * pointsC;  // [4x4] * [4xN] => [4xN]

  // 雷达坐标系转换为世界坐标系: [4xN]
  Eigen::Matrix<float, 4, Eigen::Dynamic> pointsW;
  pointsW.resize(4, n_points);
  pointsW = T_l2w * pointsL;

  // 点云输出
  cloud->points.resize(pointsW.cols());
  cloud->width = pointsW.cols();
  cloud->height = 1;
  for(int i=0; i<pointsW.cols(); i++) {
    cloud->points[i].x = pointsW(0, i);
    cloud->points[i].y = pointsW(1, i);
    cloud->points[i].z = pointsW(2, i);
  }
}


/** @brief 深度图转点云
 *  Depth, Intensity => 点云(PointXYZI)
 *  坐标系关系参考 [Depth => 点云(PointXYZ)]
 *
 * @param in depth      - 深度图, CV_32F
 * @param in intensity  - 强度值图, CV_32F, 大小与depth一致
 * @param out cloud     - 点云: PointXYZ
 * @param in K_inv      - 相机内参的逆： 3x3
 * @param in T_l2c_inv  - 雷达->相机的标定外参的逆: 4x4
 * @param in T_l2w      - 雷达安装位姿: 4x4
 */
void CommonFuns::depth2cloud(
    const cv::Mat &depth, const cv::Mat &intensity, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    const Eigen::Matrix3f &K_inv, const Eigen::Matrix4f &T_l2c_inv, const Eigen::Matrix4f &T_l2w)
{
  if (depth.empty() || intensity.empty()) {
    printf("[E:%s:%d][CommonFuns::depth2cloud] depth or intensity is empty!\n", __FILE__, __LINE__);
    return;
  }
  if (depth.size != intensity.size) {
    printf("[E:%s:%d][CommonFuns::depth2cloud] depth and intensity is not same size!\n", __FILE__, __LINE__);
    return;
  }
  if(!cloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>());
    cloud = temp;
  }

  // 统计非零个数
  int n_points = cv::countNonZero(depth);

  // 提取UV坐标点向量: [3xN]
  Eigen::Matrix<float, 3, Eigen::Dynamic> pointsUV;
  pointsUV.resize(3, n_points);
  // 提取强度值: 3xN
  Eigen::Matrix<float, 1, Eigen::Dynamic> v_intensity;
  v_intensity.resize(1, n_points);

  int point_index = 0;
  for (int u = 0; u < depth.cols; u++) {
    for (int v = 0; v <  depth.rows; v++) {
      float depth_val = depth.at<float>(v, u);
      if (u >=0 && u < depth.cols && v >=0 && v <depth.rows && depth_val>0 && point_index<n_points) {
        // UV
        pointsUV(0, point_index) = u * depth_val;
        pointsUV(1, point_index) = v * depth_val;
        pointsUV(2, point_index) = depth_val;
        // Intensity
        v_intensity(0, point_index) = intensity.at<float>(v, u);

        point_index++;
      }
    }
  }

  // UV坐标系转换相机坐标系: [3xN]
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> pointsC;
  pointsC.resize(3, n_points);  // (3xN)
  pointsC = K_inv * pointsUV;   // [3x3] * [3xN] => [3xN]

  // 相机坐标系点向量转换的齐坐标: [3xN] => [4xN]
  pointsC.conservativeResize(pointsC.rows()+1, pointsC.cols()); // [3xN] => [4xN], 添加一行 => 齐次坐标
  pointsC.row(pointsC.rows() - 1) = Eigen::MatrixXf::Ones(1, pointsC.cols());

  // 相机坐标系转换为雷达坐标系: [4xN]
  Eigen::Matrix<float, 4, Eigen::Dynamic> pointsL;
  pointsL = T_l2c_inv * pointsC;  // [4x4] * [4xN] => [4xN]

  // 雷达坐标系转换为世界坐标系: [4xN]
  Eigen::Matrix<float, 4, Eigen::Dynamic> pointsW;
  pointsW.resize(4, n_points);
  pointsW = T_l2w * pointsL;

  // 点云输出
  cloud->points.resize(pointsW.cols());
  cloud->width = pointsW.cols();
  cloud->height = 1;
  for(int i=0; i<pointsW.cols(); i++) {
    cloud->points[i].x = pointsW(0, i);
    cloud->points[i].y = pointsW(1, i);
    cloud->points[i].z = pointsW(2, i);
    cloud->points[i].intensity = v_intensity(0, i);
  }
}

// 获取颜色值
unsigned CommonFuns::get_color(int index)
{
  static unsigned int colors[] = {0xFF0000,	0x00FF00,	0x0000FF,	0xFF8C69,
                                  0xFF34B3,	0x8E388E,	0xEEB422,	0x8B6914,
                                  0xBC8F8F,	0xB3EE3A,	0xFFD39B,	0xBFEFFF,
                                  0x7CCD7C,	0x757575,	0x483D8B,	0x458B74,
                                  0xFFBBFF,	0x00FA9A,	0xEEEE00,	0xFFFFAA};
  static int colors_num = sizeof(colors)/sizeof(int);
  return colors[index % colors_num];
}

/** @brief 深度图转点云
 * RGB, Depth => Cloud:PointXYZRGB
 *  坐标系关系参考 [Depth => 点云(PointXYZ)]
 *
 * @param in rgb        - 彩色相机图像
 * @param in depth      - 深度图
 * @param in bboxes     - 检测目标bbox: [x1_min, y1_min, x1_max, y1_max, ....]
 * @param out cloud     - 点云(PointXYZRGB)
 * @param in K_inv      - 相机内参的逆: 3x3
 * @param in T_l2c_inv  - 雷达-相机外参的逆: 4x4
 * @param in T_l2w      - 雷达位姿: 4x4
 */
void CommonFuns::depth2cloud(
    const cv::Mat &rgb, const cv::Mat &depth, const std::vector<int> &bboxes,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const Eigen::Matrix3f &K_inv, const Eigen::Matrix4f &T_l2c_inv, const Eigen::Matrix4f &T_l2w)
{
  static unsigned int colors[] = {0xFF0000,	0x00FF00,	0x0000FF,	0xFF8C69,
                                  0xFF34B3,	0x8E388E,	0xEEB422,	0x8B6914,
                                  0xBC8F8F,	0xB3EE3A,	0xFFD39B,	0xBFEFFF,
                                  0x7CCD7C,	0x757575,	0x483D8B,	0x458B74,
                                  0xFFBBFF,	0x00FA9A,	0xEEEE00,	0xFFFFAA};
  static int colors_num = sizeof(colors)/sizeof(int);

  if (rgb.empty() || depth.empty()) {
    printf("[E:%s:%d][CommonFuns::depth2cloud] rgb or depth is empty!\n", __FILE__, __LINE__);
    return;
  }
  if(!cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud = temp;
  }

  // 提取UV坐标点向量: [3xN]
  Eigen::Matrix<float, 3, Eigen::Dynamic> pointsUV;
  Eigen::Matrix<int, 3, Eigen::Dynamic> pointsRGB;
  Eigen::Matrix<int, 1, Eigen::Dynamic> obj_masks;
  int n_points = cv::countNonZero(depth);
  pointsUV.resize(3, n_points);   // (3xN)
  pointsRGB.resize(3, n_points);  // (3xN)
  obj_masks.resize(1, n_points);  // (1xN)
  obj_masks.setZero();
  int point_index = 0;
  for (int u = 0; u < depth.cols; u++) {
    for (int v = 0; v <  depth.rows; v++) {
      float depth_val = depth.at<float>(v, u);
      if (u >=0 && u < depth.cols && v >=0 && v <depth.rows && depth_val>0 && point_index<n_points) {

        pointsUV(0, point_index) = u * depth_val;
        pointsUV(1, point_index) = v * depth_val;
        pointsUV(2, point_index) = depth_val;

        pointsRGB(0, point_index) = rgb.at<cv::Vec3b>(v, u)[0];
        pointsRGB(1, point_index) = rgb.at<cv::Vec3b>(v, u)[1];
        pointsRGB(2, point_index) = rgb.at<cv::Vec3b>(v, u)[2];

        // 目标点云区域标记
        for(unsigned int k = 0; k<bboxes.size()/4; k++) {
          int x1 = bboxes[k*4+0];
          int y1 = bboxes[k*4+1];
          int x2 = bboxes[k*4+2];
          int y2 = bboxes[k*4+3];
          if(u>=x1 && u<=x2 && v>=y1 && v<=y2) {
            obj_masks(0, point_index) = k+1;
          }
        }

        point_index++;
      }
    }
  }
  if (n_points != point_index) {
    n_points = point_index;
    pointsUV.conservativeResize(pointsUV.rows(), n_points);
    pointsRGB.conservativeResize(pointsRGB.rows(), n_points);
    obj_masks.conservativeResize(obj_masks.rows(), n_points);
  }

  // UV坐标系转换相机坐标系: [3xN]
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> pointsC;
  pointsC.resize(3, n_points);  // (3xN)
  pointsC = K_inv * pointsUV;   // [3x3] * [3xN] => [3xN]

  // 相机坐标系下的齐次坐标: [3xN] => [4xN]
  pointsC.conservativeResize(pointsC.rows()+1, pointsC.cols()); // [3xN] => [4xN], 添加一行 => 齐次坐标
  pointsC.row(pointsC.rows() - 1) = Eigen::MatrixXf::Ones(1, pointsC.cols());

  // 相机坐标系转换为雷达坐标系: [4xN]
  Eigen::Matrix<float, 4, Eigen::Dynamic> pointsL;
  pointsL = T_l2c_inv * pointsC;  // [4x4] * [4xN] => [4xN]

  // 转换为世界坐标点向量: [4xN]
  Eigen::Matrix<float, 4, Eigen::Dynamic> pointsW;
  pointsW.resize(4, n_points);
  pointsW = T_l2w * pointsL;

  // 点云输出
  cloud->points.resize(pointsW.cols());
  cloud->width = pointsW.cols();
  cloud->height = 1;
  for(int i=0; i<n_points; i++) {
    cloud->points[i].x = pointsW(0, i);
    cloud->points[i].y = pointsW(1, i);
    cloud->points[i].z = pointsW(2, i);
    if (isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z)) {
      cloud->points[i].x = 0;
      cloud->points[i].y = 0;
      cloud->points[i].z = 0;
    }

    if (obj_masks(0, i)>0) {
      int color = colors[(obj_masks(0,i)-1)%colors_num];
      cloud->points[i].r = (color & 0xFF0000)>>16;
      cloud->points[i].g = (color & 0x00FF00)>>8;
      cloud->points[i].b = (color & 0x0000FF)>>0;
    } else {
      cloud->points[i].r = pointsRGB(0, i);
      cloud->points[i].g = pointsRGB(1, i);
      cloud->points[i].b = pointsRGB(2, i);
    }
  }
}


/** @brief RGB2CLOUD投影测试
 * image, cloud_in:PointXYZ => cloud_out: PointXYZRGB
 * @param in image - 彩色相机图像
 * @param in cloudPtr - 输入点云(PointXYZ)
 * @param out colorCloud - 输出点云(PointXYZRGB)
 * @param in K - 相机内参: 3x3
 * @param in D - 相机畸变系数: 1x5
 * @param in T_l2c - 雷达-相机外参: 4x4
 */
void CommonFuns::rgb2cloud(
    const cv::Mat &image, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloudPtr,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colorCloud,
    const cv::Mat &K, const cv::Mat & D, const cv::Mat &T_l2c)
{
    if (image.empty() || !cloudPtr) {
      printf("[E:%s:%d][CommonFuns::rgb2cloud] image or cloudPtr is null!\n", __FILE__, __LINE__);
      return;
    }
    if (!colorCloud) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>());
      colorCloud = temp;
    }

    cv::Mat T_l2c_inv = T_l2c.inv();

    // 去畸变
    cv::Mat rec_left, undistorted_img;
    cv::undistort(image, undistorted_img, K, D);

    // 提取点云坐标
    std::vector<cv::Point3f> pointsL;  // 3D lidar points
    for (unsigned int i=0; i<=cloudPtr->points.size(); i++)
    {
        pointsL.push_back(cv::Point3f(cloudPtr->points[i].x, cloudPtr->points[i].y, cloudPtr->points[i].z));
    }

    //获取深度图
    cv::Mat depthImage = cv::Mat::zeros(undistorted_img.size(), CV_32FC1); //深度图, Create depth image --- added by Pop
    std::vector<cv::Point2f> pointsUV;   // 深度图投影下的UV坐标序列，Projected 2D points
    for (unsigned int i = 0; i < pointsL.size(); i++)
    {
        // convert "pointsL" to homogeneous "ptMat"
        cv::Point3f pt = pointsL[i];
        cv::Mat ptMat; // 雷达坐标系的齐次坐标
        ptMat = (cv::Mat_<float>(4, 1) << pt.x, pt.y, pt.z, 1);

        // Perform matrix multiplication and save as Mat_ for easy element access
        cv::Mat dstMat;
        cv::Mat Ka; // 相机内参增广矩阵: [K, 0]
        Ka = (cv::Mat_<float>(3,4) <<
                K.at<float>(0, 0), K.at<float>(0, 1), K.at<float>(0, 2), 0,
                K.at<float>(1, 0), K.at<float>(1, 1), K.at<float>(1, 2), 0,
                K.at<float>(2, 0), K.at<float>(2, 1), K.at<float>(2, 2), 0
                );
        dstMat= Ka * T_l2c * ptMat;      // UV投影zc*(u, v, 1): pointsL -> pointsC -> pointsUV

        // Divide first 2 elements by the 3rd and assign to Point2f
        float scale = dstMat.at<float>(2, 0); // 深度值
        // if (scale < 10e-6)
        //     scale  = 10e-6;
        cv::Point2f dst(dstMat.at<float>(0, 0) / scale, dstMat.at<float>(1, 0) / scale); // (U, V)

        // ------- added by Pop ----------//
        // 边界检测
        if ( dst.x >= 0 && dst.y >= 0 && dst.x < depthImage.cols && dst.y < depthImage.rows)
        {
            // cout << "dst[" << i << "]=" << dst.y << " " << dst.x << endl;
            depthImage.at<float>(dst.y, dst.x) = pt.x;  // should be pt.x, since in Velo, pt.x is depth infomation
        }
        pointsUV.push_back(dst);
    }

    // 获取RGB的3D投影
    pcl::PointXYZRGB pt_c;
    for(int v=0; v<undistorted_img.rows; v+=1)
    {
        for(int u=0; u<undistorted_img.cols; u+=1) // from left_image to undistorted_img
        {
            float depth = depthImage.at<float>(v,u);

            // assign value to colored point cloud one by one
            if (depth > 0)
            {
                // UV坐标系 -> 相机坐标系
                pt_c.x = (u - K.at<float>(0,2)) * depth / K.at<float>(0,0);
                pt_c.y = (v - K.at<float>(1,2)) * depth / K.at<float>(1,1);
                pt_c.z = depth;
                // 相机坐标系 -> 雷达坐标系
                cv::Mat ptC; // 雷达坐标系的齐次坐标
                ptC = (cv::Mat_<float>(4, 1) << pt_c.x, pt_c.y, pt_c.z, 1);
                cv::Mat ptL;
                ptL = T_l2c_inv * ptC;

                pt_c.x = ptL.at<float>(0);
                pt_c.y = ptL.at<float>(1);
                pt_c.z = ptL.at<float>(2);
                // 雷达坐标系 -> 世界坐标系
                pt_c.b = undistorted_img.at<cv::Vec3b>(v,u)[0];	// changed from inputImage1_ -> image1_undistort, .r -> .b
                pt_c.g = undistorted_img.at<cv::Vec3b>(v,u)[1];
                pt_c.r = undistorted_img.at<cv::Vec3b>(v,u)[2];
            }
            else
            {
                continue;
            }

            if (!pcl_isfinite (pt_c.x) ||
                !pcl_isfinite (pt_c.y) ||
                !pcl_isfinite (pt_c.z) )
                continue;

            colorCloud->points.push_back (pt_c);
            // tmpMat.push_back(tmpVec);
        }
    }

    // 发布RGB的3D投影
    colorCloud->header.frame_id = "livox_frame";
    colorCloud->height = 1;
    colorCloud->width = colorCloud->size();
}

/** @brief RGB2CLOUD投影测试
 *  BUG: 存在Nan问题导致rviz异常.
 * image, cloud_in:PointXYZ => cloud_out: PointXYZRGB
 * @param in image - 彩色相机图像
 * @param in cloudPtr - 输入点云(PointXYZ)
 * @param out colorCloud - 输出点云(PointXYZRGB)
 * @param in K - 相机内参: 3x3
 * @param in D - 相机畸变系数: 1x5
 * @param in T_l2c - 雷达-相机外参: 4x4
 */
void CommonFuns::rgb2cloud(
    const cv::Mat &image, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colorCloud,
    const Eigen::Matrix3f &K, const Eigen::Matrix<float, 1, 5> &D,
    const Eigen::Matrix4f &T_l2c, const Eigen::Matrix4f &T_l2w)
{
    if (image.empty() || !cloud) {
      printf("[E:%s:%d][CommonFuns::rgb2cloud] image or cloud is empty!\n", __FILE__, __LINE__);
      return;
    }
    if(!colorCloud) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>());
      colorCloud = temp;
    }

    cv::Mat cv_K, cv_D;
    cv::eigen2cv(K, cv_K);
    cv::eigen2cv(D, cv_D);

    Eigen::Matrix<float, 3, 4> Ka;
    Ka.block(0, 0, 3, 3) = K;
    Ka.col(3) = Eigen::Vector3f::Zero();

    Eigen::Matrix3f K_inv = K.inverse();
    Eigen::Matrix4f T_l2c_inv = T_l2c.inverse();

    // 去畸变
    cv::Mat rec_left, undistorted_img;
    cv::undistort(image, undistorted_img, cv_K, cv_D);

    // 点云到深度图投影
    cv::Mat depth = cv::Mat::zeros(image.rows, image.cols, CV_32F);
    cv::Mat intensity = cv::Mat::zeros(image.rows, image.cols, CV_32F);
    cloud2depth(cloud, depth, intensity, K, Ka, T_l2c,  1.0,  300.0);

    // (深度图+彩色图) 到 点云投影
    std::vector<int> bboxes;
    depth2cloud(undistorted_img, depth, bboxes, colorCloud, K_inv, T_l2c_inv, T_l2w);

    // 发布RGB的3D投影
    colorCloud->header.frame_id = "livox_frame";
    colorCloud->height = 1;
    colorCloud->width = colorCloud->size();
}

/** @brief 保存测试数据
 * @param in spath - 数据保存路径
 * @param in frame_index - 数据帧序号
 * @param in img - 彩色图
 * @param in depth - 深度图
 * @param in intensity - 强度值图
 * @param in class_ids - 类别ID向量表
 * @param in class_names - 类别名称向量表
 * @param in bboxes - 目标bbox向量表
 * @param in scores - 目标置信度向量表
 * @param in track_ids - 目标跟踪ID向量表
 */
void CommonFuns::save_data(
    const std::string &spath, int frame_index,
    const cv::Mat &img,
    const cv::Mat &depth,
    const cv::Mat &intensity,
    const std::vector<int> &class_ids,
    const std::vector<std::string> &class_names,
    const std::vector<int> &bboxes,
    const std::vector<float> &scores,
    const std::vector<int> &track_ids)
{
  char img_file [255] = {0};
  char depth_file [255] = {0};
  char data_file [255] = {0};
  sprintf(  img_file, "%s/%06d_img.jpg", spath.c_str(), frame_index);
  sprintf(depth_file, "%s/%06d_depth.jpg", spath.c_str(), frame_index);
  sprintf( data_file, "%s/%06d_data.xml", spath.c_str(), frame_index);

  // 保存图像
  cv::Mat depth_;
  cv::normalize(depth, depth_, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  cv::imwrite(depth_file, depth_);
  cv::imwrite(img_file, img);

  // 保存数据
  cv::FileStorage fs(data_file, cv::FileStorage::WRITE);
  fs << "depth" << depth;              // 深度图
  fs << "intensity" << intensity;      // 强度值图
  fs << "class_ids" << class_ids;      // MOT->class_ids
  fs << "class_names" << class_names;  // MOT->class_names
  fs << "bboxes" << bboxes;            // MOT->bboxes
  fs << "scores" << scores;            // MOT->scores
  fs << "track_ids" << track_ids;      // MOT->track_ids
  fs.release();
}


/** @brief 加载测试数据
 * @param in spath - 数据保存路径
 * @param in frame_index - 数据帧序号
 * @param out img - 彩色图
 * @param out depth - 深度图
 * @param out intensity - 强度值图
 * @param out class_ids - 类别ID向量表
 * @param out class_names - 类别名称向量表
 * @param out bboxes - 目标bbox向量表
 * @param out scores - 目标置信度向量表
 * @param out track_ids - 目标跟踪ID向量表
 */
void CommonFuns::read_data(
    const std::string &spath,
    int frame_index,
    cv::Mat &img,
    cv::Mat &depth,
    cv::Mat &intensity,
    std::vector<int> &class_ids,
    std::vector<std::string> &class_names,
    std::vector<int> &bboxes,
    std::vector<float> &scores,
    std::vector<int> &track_ids)
{
  char img_file [255] = {0};
  char data_file [255] = {0};
  sprintf(img_file, "%s/%06d_img.jpg", spath.c_str(), frame_index);
  sprintf(data_file, "%s/%06d_data.xml", spath.c_str(), frame_index);

  img = cv::imread(img_file);

  cv::FileStorage fs(data_file, cv::FileStorage::READ);
  fs["depth"] >> depth;              // 深度图
  fs["intensity"] >> intensity;      // 强度值图
  fs["class_ids"] >> class_ids;      // MOT->class_ids
  fs["class_names"] >> class_names;  // MOT->class_names
  fs["bboxes"] >> bboxes;            // MOT->bboxes
  fs["scores"] >> scores;            // MOT->scores
  fs["track_ids"] >> track_ids;      // MOT->track_ids
  fs.release();
}

/** @brief 加载测试数据
 * @param in spath - 数据保存路径
 * @param in frame_index - 数据帧序号
 * @param out img - 彩色图
 * @param out depth - 深度图
 * @param out intensity - 强度值图
 */
void CommonFuns::read_data(
    const std::string &spath,
    int frame_index,
    cv::Mat &img,
    cv::Mat &depth,
    cv::Mat &intensity)
{
  char img_file [255] = {0};
  char data_file [255] = {0};
  sprintf(img_file, "%s/%06d_img.jpg", spath.c_str(), frame_index);
  sprintf(data_file, "%s/%06d_data.xml", spath.c_str(), frame_index);

  img = cv::imread(img_file);

  cv::FileStorage fs(data_file, cv::FileStorage::READ);
  fs["depth"] >> depth;              // 深度图
  fs["intensity"] >> intensity;      // 强度值图
  fs.release();
}

/** @brief 点云坐标点统计
 */
void CommonFuns::points_stat(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    Eigen::Vector4f &min_val,
    Eigen::Vector4f &max_val,
    Eigen::Vector4f &mean_val)
{
  Eigen::Matrix<float, 4, Eigen::Dynamic> points;
  int n_points = cloud->points.size();
  points.resize(4, n_points);
  for (int i = 0; i<n_points; i++) {
    points(0, i) = cloud->points[i].x;
    points(1, i) = cloud->points[i].y;
    points(2, i) = cloud->points[i].z;
    points(3, i) = cloud->points[i].intensity;
  }
  min_val = points.rowwise().minCoeff();
  max_val = points.rowwise().maxCoeff();
  mean_val = points.rowwise().mean();
}

/** @brief 点云着色
 * 默认白色，其他点云块根据 cluster_indices 块序号从 get_color获取颜色值
 * @param cloud_in in 输入点云[PointXYZI]
 * @param cluster_indices in 点云聚类块
 * @param cloud_out out 输出点云[PointXYZRGB]
 */
void CommonFuns::cloud_mask_color(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in,
    const std::vector<pcl::PointIndices> &cluster_indices,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
  // check input
  if (!cloud_in) {
    printf("[W:%s:%d][cloud_mask_color] cloud_in is NULL!\n", __FILE__, __LINE__);
  }
  if (!cloud_out) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_out = cloud;
  }

  // init
  cloud_out->clear();
  cloud_out->width = cloud_in->width;
  cloud_out->height = cloud_in->height;
  for(unsigned int i = 0; i<cloud_in->points.size(); i++) {
    pcl::PointXYZRGB point;
    point.x = cloud_in->points[i].x;
    point.y = cloud_in->points[i].y;
    point.z = cloud_in->points[i].z;
    point.r = 255;
    point.g = 255;
    point.b = 255;
    cloud_out->points.push_back(point);
  }

  // mask color
  for (unsigned int i = 0; i<cluster_indices.size(); i++) {
    for (unsigned  int j = 0; j<cluster_indices[i].indices.size(); j++) {
      cloud_out->points[cluster_indices[i].indices[j]].r = (get_color(i) & 0xFF0000)>>16;
      cloud_out->points[cluster_indices[i].indices[j]].g = (get_color(i) & 0x00FF00)>>8;
      cloud_out->points[cluster_indices[i].indices[j]].b = (get_color(i) & 0x0000FF)>>0;
    }
  }
}

/** @brief 点云着色
 * @param cloud_in in 输入点云[PointXYZI]
 * @param color in 指定点云颜色
 * @param cloud_out out 输出点云[PointXYZRGB]
 */
void CommonFuns::cloud_mask_color(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in,
    int color,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
  // check input
  if (!cloud_in) {
    printf("[W:%s:%d][cloud_mask_color] cloud_in is NULL!\n", __FILE__, __LINE__);
  }
  if (!cloud_out) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_out = cloud;
  }

  // mask color
  cloud_out->clear();
  cloud_out->width = cloud_in->width;
  cloud_out->height = cloud_in->height;
  for(unsigned int i = 0; i<cloud_in->points.size(); i++) {
    pcl::PointXYZRGB point;
    point.x = cloud_in->points[i].x;
    point.y = cloud_in->points[i].y;
    point.z = cloud_in->points[i].z;
    point.r = (color & 0xFF0000) >> 16;
    point.g = (color & 0x00FF00) >> 8;
    point.b = (color & 0x0000FF) >> 0;
    cloud_out->points.push_back(point);
  }
}

/** @brief 提取目标点云
 *  坐标变换图
 *    UV <--K-- Camera <--T_l2c-- Livox --T_l2w--> World  [v]
 *
 *    符号"--->"：   刚体 --T--> 参考坐标系
 *
 *    World : 世界坐标系
 *    Livox : Livox坐标系
 *    Camera: 相机坐标系
 *    UV    : 像素坐标系
 *
 *  坐标变换参数
 *    K    : 相机内参
 *    T_l2c: 相机-雷达外参标定，参考系为相机坐标系, 如果忽略相机-雷达安装偏差，可以近似pose=(0, 0, 0, pi/2, -pi/2, 0)
          相机坐标系(参考系)                                                     Livox坐标系
                                          (pi/2, -pi/2, 0)

                                          ^ z                                         ^ z
                + z                       |     + y                                   |     + x
              .                           |   .                                       |   .
            .                             | .                                         | .
          o-----------> x    RotX(pi/2)   o-----------> x     RotY(-pi/2)     <-------o
          |                  --------->                       ---------->     y
          |
          |
          v y
 *    T_l2w: 相机套件位姿(Livox坐标系)齐次矩阵,, 参考系为世界坐标
 *
 *  处理流程
 *    obj_depth <- depth[bbox]                  // 提取目标深度图区域
 *    cvPointsUV <- extract(obj_depth, u0, v0)  // 提取目标UV坐标向量, [3xN]
 *    cvPointsC <- K_inv * cvPointsUV           // 相机坐标系点向量, [3xN]
 *    cvPointsCa <- [cvPointsC; ones]           // 相机坐标系的齐次坐标, [3x4] => [4xN]
 *    cvPointsL <- T_l2c_inv * cvPointsCa       // Livox坐标系点向量，[4xN]
 *    cvPointsW <- T_l2w * cvPointsL            // 世界坐标系点向量, [4xN]
 *
 * @param depth in - [Mat ]全局深度图
 * @param intensity in - [Mat ] 强度值图
 * @param bbox in  - 检测目标边框, [x1, y1, x2, y2] <-> [left, top, right, bottom], x<->u, y<->v
 * @param K_inv in - 相机内参的逆, 3x3
 * @param T_l2c_inv in - 雷达-相机外参标定的逆, 4x4
 * @param T_l2w in - 套件安装位姿, 4x4
 * @param out points - [pcl::PointCloud]目标点云
 */
void CommonFuns::extract_obj(
    const cv::Mat &depth, const cv::Mat &intensity, const Eigen::Vector4i &bbox,
    const Eigen::Matrix3f &K_inv, const Eigen::Matrix4f &T_l2c_inv, const Eigen::Matrix4f &T_l2w,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &points)
{
  assert(bbox[0]>=0 && bbox[2]<depth.cols && bbox[1]>=0 && bbox[3]<depth.rows);

  // 空指针自动创建
  if (!points) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>());
    points = temp;
  }

  // 提取深度图的目标区域
  const cv::Mat obj_depth(depth, cv::Range(bbox[1], bbox[3]), cv::Range(bbox[0], bbox[2]));
  const cv::Mat obj_intensity(intensity, cv::Range(bbox[1], bbox[3]), cv::Range(bbox[0], bbox[2]));

  // 提取UV坐标点向量: [3xN]
  Eigen::Matrix<float, 3, Eigen::Dynamic> pointsUV;
  Eigen::Matrix<float, 1, Eigen::Dynamic> vIntensity;
  int n_points = cv::countNonZero(obj_depth); // 统计非0个数用于构造Matrix数组
  if (n_points<=0) {
    printf("[W:%s:%d][extract_obj] no points in bbox!\n", __FILE__, __LINE__);
    return;
  }
  pointsUV.resize(3, n_points);   // (3xN)
  vIntensity.resize(1, n_points); // (1xN)
  int point_index = 0;
  for (int u = bbox[0]; u < bbox[2]; u++) {
    for (int v = bbox[1]; v <  bbox[3]; v++) {
      // float depth_val = depth.at<float>(v, u);
      float depth_val = depth.at<float>(v, u) * 0.001;
      if (u >=0 && u < depth.cols && v >=0 && v <depth.rows && depth_val>0 && point_index<n_points) {
        // UV
        pointsUV(0, point_index) = u * depth_val;
        pointsUV(1, point_index) = v * depth_val;
        pointsUV(2, point_index) = depth_val;
        // Intensity
        vIntensity(0, point_index) = intensity.at<float>(v, u);

        point_index++;
      }
    }
  }

  // 转换相机坐标系点向量: [3xN]
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> pointsC;
  pointsC.resize(3, n_points); // (3xN)
  pointsC = K_inv * pointsUV; // [3x3] * [3xN] => [3xN]

  // 相机坐标系点向量转换为其次坐标: [3xN] => [4xN]
  pointsC.conservativeResize(pointsC.rows()+1, pointsC.cols()); // [3xN] => [4xN], 添加一行单位向量 => 齐次坐标
  pointsC.row(pointsC.rows() - 1) = Eigen::MatrixXf::Ones(1, pointsC.cols());

  // 转换为Livox坐标系点向量: [4xN]
  Eigen::Matrix<float, 4, Eigen::Dynamic> pointsL;
  pointsL = T_l2c_inv * pointsC;  // [4x4] * [4xN] => [4xN]

  // 转换为世界坐标点向量: [4xN]
  Eigen::Matrix<float, 4, Eigen::Dynamic> pointsW;
  pointsW.resize(4, n_points);
  pointsW = T_l2w * pointsL;

  // 点云输出
  points->points.resize(pointsW.cols());
  points->width = pointsW.cols();
  points->height = 1;
  for(int i=0; i<pointsW.cols(); i++) {
    points->points[i].x = pointsW(0, i);
    points->points[i].y = pointsW(1, i);
    points->points[i].z = pointsW(2, i);
    points->points[i].intensity = vIntensity(0, i);
  }
}

/** @brief 提取点云
 * @param cloud_in in - 输入点云
 * @param indices in - 提取点云索引
 * @param cloud_out out - 输出点云
 */
void CommonFuns::extract_points(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in,
                           const pcl::PointIndices::ConstPtr indices,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out)
{
  // 点云提取对象
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  // 提取内点
  extract.setInputCloud(cloud_in);
  extract.setNegative(false);       // 提取内点
  extract.setIndices(indices);     // 点索引
  extract.filter(*cloud_out);       // 提取
}

/** @brief 地面过滤
 * @param obj_index in - 目标序号
 * @param cloud in - 输入点云
 * @param d_threshold in - 地面过滤距离阈值, 小于阈值视为平面内点, e.g. 0.05(m)
 * @param n_threshold in - 地面过滤法线阈值, 小于阈值视为匹配的地面, e.g. 0.9
 * @param min_size in - 地面过滤最少点数，小于阈值忽略地面过滤，直接返回， e.g. 10
 * @param verbose in - 调试等级
 * @param cloud_filter out - 输出点云, 已剔除地面点云
 */
void CommonFuns::filter_ground(
    int obj_index,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    float d_threshold, float n_threshold, float percent, int min_size,
    int verbose,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_filter)
{
  if (!cloud) {
    printf("[E:%s:%d][CommonFuns::filter_ground] cloud is empty!\n", __FILE__, __LINE__);
    return;
  }
  if ((int)cloud->points.size()<min_size) {
    printf("[E:%s:%d][CommonFuns::filter_ground] cloud's points is %ld, less then %d!\n", __FILE__, __LINE__,
           cloud->points.size(), min_size);
    return;
  }
  if (!cloud_filter) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>());
    cloud_filter = temp;
  }

  if (verbose>0) {
    printf("[I:%s:%d][filter_ground] params d_threshold=%f, n_threshold=%f, percent=%f, min_size=%d\n",
           __FILE__, __LINE__, d_threshold, n_threshold, percent, min_size);
  }

  cloud_filter->clear();
  // 临时缓存
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());

  // 剩余未处理点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZI>());
  *cloud_remain = *cloud;

  // 点云提取对象
  pcl::ExtractIndices<pcl::PointXYZI> extract;

  // 创建模型参数[a, b, c, d]: ax+by+cz+d=0
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // 创建内点存储对象: 拟合平面
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // 创建分割对象
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // 可选设置: 优化模型参数
  seg.setOptimizeCoefficients(true);
  // 必须设置: 分割模型类型(平面)
  seg.setModelType(pcl::SACMODEL_PLANE);
  // 设置参数估计方法
  seg.setMethodType(pcl::SAC_RANSAC);
  // 设置距离阈值
  seg.setDistanceThreshold(d_threshold);

  // 原始点数
  int n_points = (int)cloud_remain->points.size();
  // 迭代过滤平面
  int iter_index = 0;
  while(cloud_remain->points.size() > percent * n_points && (int)cloud_remain->points.size()>min_size)
  {
    // 输入点云
    seg.setInputCloud(cloud_remain);
    // 分割
    seg.segment(*inliers, *coefficients);
    if ((int)inliers->indices.size() <= 0) {
        PCL_ERROR("[CommonFuns::filter_ground]Could not estimate a planar model for the given dataset.");
        break;
    }

    // 保存不满足地面条件的平面点云(最后补回到输出点云): 法线量（x, y, z）-> |z|<0.9
    if (fabs(coefficients->values[2])<n_threshold) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>());
      extract.setInputCloud(cloud_remain);
      extract.setIndices(inliers);
      extract.setNegative(false); // 提取内点: 平面点
      extract.filter(*cloud_plane);

      *cloud_filter += *cloud_plane;
    }

    if (verbose>1) {
      printf("[I:%s:%d][filter_ground][iter %d] %ld points in cloud_remain and %ld points in plane \n"
             "    plane coeff: (a=%f, b=%f, c=%f, d=%f)\n"
             "         params: d_threshold=%f, n_threshold=%f\n"
             "      is_ground: %d\n",
             __FILE__, __LINE__, iter_index,
             cloud_remain->points.size(), inliers->indices.size(),
             coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3],
             d_threshold, n_threshold, fabs(coefficients->values[2])>=n_threshold);
      if (verbose>2) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr obj_plane(new pcl::PointCloud<pcl::PointXYZI>());
        CommonFuns::extract_points(cloud_remain, inliers, obj_plane);
        {
          Eigen::Vector4f min_val, max_val, mean_val;
          CommonFuns::points_stat(obj_plane, min_val, max_val, mean_val);
          printf("[I:%s:%d][estimate_3dbboxs] plane stat: %ld points \n"
                 "     min(%f, %f, %f, %f)\n"
                 "     max(%f, %f, %f, %f)\n"
                 "    mean(%f, %f, %f, %f)\n", __FILE__, __LINE__,
                 obj_plane->points.size(),
                 min_val(0), min_val(1), min_val(2), min_val(3),
                 max_val(0), max_val(1), max_val(2), max_val(3),
                 mean_val(0), mean_val(1), mean_val(2), mean_val(3));
        }

        char pcd_file[255] = {};
        sprintf(pcd_file, "/tmp/obj_cloud_plane_%d_%d.pcd", obj_index, iter_index);
        pcl::io::savePCDFile(pcd_file, *obj_plane);
        printf("    save to '%s'\n", pcd_file);
      }
    }
    if (inliers->indices.size() < cloud_remain->points.size()) {
      // 把匹配的平面从点云中清除
      extract.setInputCloud(cloud_remain);
      extract.setIndices(inliers);
      extract.setNegative(true);      // 提取外点
      extract.filter(*cloud_temp);    // 剩下未处理点云
      cloud_remain.swap(cloud_temp);  // swap: cloud_filter <-> cloud_remain
    } else {
      // 所有点都处理完了
      cloud_remain->clear();
    }

    iter_index ++;
  }

  // 返回滤除地面后的点云: 剩余点云+非地面的平面点云
  if (cloud_remain->points.size()>0) {
    *cloud_filter += *cloud_remain;
  }
  if (verbose>0) {
    printf("[I:%s:%d][filter_ground] remain %ld points after filter ground!\n",
           __FILE__, __LINE__, cloud_filter->points.size());
  }
}


/** @brief 欧氏分割
 * @param cloud in - 输入点云
 * @param cluster_indices out - 输出聚类结果
 * @param tole in - 邻近点阈值
 * @param min_size - 最少点数
 * @param max_size - 最多点数
 */
void CommonFuns::segment_euclidean_cluster(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    std::vector<pcl::PointIndices> &cluster_indices,
    float tole, int min_size, int max_size)
{
  if (!cloud || cloud->empty() || cloud->points.size()<=0) {
    printf("[W:%s:%d][segment_euclidean_cluster] cloud is empty!\n", __FILE__, __LINE__);
    return ;
  }

  // 点云聚类为了便于后面的平面分割
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclidean_cluster;
  euclidean_cluster.setClusterTolerance(tole);        // 设置近邻搜索半径为5ｃｍ, 两个相邻点距离小于阈值归为一类
  euclidean_cluster.setMinClusterSize(min_size);
  euclidean_cluster.setMaxClusterSize(max_size);
  euclidean_cluster.setSearchMethod(tree);
  euclidean_cluster.setInputCloud(cloud);
  euclidean_cluster.extract(cluster_indices);
}


#endif // COMMON_H
