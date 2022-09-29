#ifndef OBJ_CLOUD_PATTERN_H
#define OBJ_CLOUD_PATTERN_H

/** @brief 模板匹配目标区域
 * 算法应用场景
 * ==========
 *  应用前置条件
 *  1. 点云已转换到世界坐标系
 *  2. 点云已做预处理: 噪声过滤，地面过滤，下采样等
 *  3. 处理点云仅限2D目标框内的区域
 *
 *  场景描述
 *  1. 点云的获取: 点云投影到深度图， 然后通过2D目标框截取深度图区域，再复原为3D点云.
 *  2. 在平视视角下，所获得的点云信息含有复杂的背景信息，可能出现的场景如下:
 *    2.1. 地面噪声
 *    2.2. 被前面物体遮挡
 *    2.3. 与后面物体重叠
 *    2.4. 周围环境散射
 *    2.5. 其他噪声影响
 *
 * 基于欧氏分割的模板匹配选择目标区域算法构想
 * ===================================
 *  假设条件:
 *    1. 每个类别都有预知合理大小(L, W, H), 在欧氏分割里已经应用了该假设估算欧氏分割阈值.
 *    2. 已经利用欧氏分割对点云进行聚类操作.
 *  算法目的:
 *    从欧氏聚类序列里选取属于目标区域的子列，并估算位姿.
 *  算法构想：
 *    把类别大小设置为一个长方体模板，以某个聚类块为参考，把邻近的聚类块聚合，然后计算与模板的匹配度。
 *    匹配度最高的聚合就是选择区域.
 *
 * 求解
 * ====
 *  从聚类序列中选取子列表征目标区域，并估算位姿
 *
 * 输入
 * ====
 *  cloud           - 2DBBox内的点云, 输入点云已做预处理
 *  cluster_indices - 聚类结果(欧氏分割), 按聚类块点数降序排序
 *  (H, W, L)       - 类别尺寸（m）
 *  w_matching      - 统计量权重: 匹配度
 *  w_distance      - 统计量权重: 距离
 *  w_points        - 统计量权重: 点数
 *  gather_coef     - 聚合阈值系数： R=max(L, W, H) / 4 * coef, D=max(L,W,H) / 2 * coef
 *  pose_type       - 位姿输出方法: 0-由PCA计算，1-聚合点云边界估算, 2-聚类首元点云边界估算(外部模块实现)
 *  norm_type       - 统计量正则化方法: 0-最大值比值法， 1-总数比值法
 *                      0-最大值比值法， 最大值项正则值为1， 其他项正则值为与最大值的比值
 *                      1-总数比值法, 累积所有项值的和， 每一项的正则值为项值与总和的比值
 *
 * 输出
 * ====
 *  3dbbox - (x, y, z, R, P, Y, L, W, H)
 *           x, y, z: 中心坐标
 *           R, P, Y: 姿态
 *           L, W, H: 长宽高(x, y, z)
 *
 * 结构体
 * ======
 *  聚类块状态
 *    struct cluster_stat
 *    {
 *      int cluster_index;                              // 聚类块索引
 *      float min_x, min_y, min_z, max_x, max_y, max_z; // 边界
 *    };
 *
 *  聚合信息结构体
 *    struct gather_info
 *    {
 *      vector<int> block_indexs; // 聚类块索引
 *      bbox3d[9];                // 聚合点云位姿估算
 *      int points;               // 计量信息: 聚合点云点数量
 *      float distance;           // 计量信息: 距离
 *      float matching;           // 计量信息: 匹配度
 *      float points_norm;        // 计量信息归一化: 聚合点云点数量
 *      float distance_norm;      // 计量信息归一化: 距离
 *      float matching_norm;      // 计量信息归一化: 匹配度
 *      score;                    // 聚合评分
 *    };
 *
 * 边缘距离阈值
 * ==========
 *         |<--R-->|
 * |-------|xxxxxxx|---o---|xxxxxxx|-------|
 *
 *             |<-------D------>|
 * |-----------|xxxxxxxxxxxxxxxx|----------|
 *
 * 聚合方法
 * =======
 *  聚合方法1(半径聚合法)
 *  ------------------
 *    obj
 *    |<-------R--------o---------------->|
 *      c_i            c_k           c_j
 *    |------|xxxxxx|-------|xxxxx|-------|
 *
 *    聚合半径 R = max(L, W, H)*2/3
 *    参考聚类块 c_k
 *    聚合子列 block_indexs
 *    遍历聚类块状态序列 cluster_stats
 *      计算聚类块 c_i 与 c_k 的距离 d(中心距离或边缘距离)
 *      如果 d < R 则把 c_i 添加到 block_indexs
 *
 *  聚合方法2(偏旁聚合法)
 *
 *                     i
 *               o     ^    o
 *            o        |       o
 *          o     [1] thr_w  [4] o
 *                  .  |   .       o
 *         o-------.---k----.-------o
 *          o       .  |  .        o
 *                [2]  .     [3]  o
 *              o      |      o
 *                     o
 *
 *    聚合偏旁宽度 W = max(L, W, H)
 *    参考聚类块 c_k
 *    划分四个区域: [1],[2],[3],[4]
 *      聚合子列 block_indexs
 *      遍历聚类块状态序列 cluster_stats
 *        计算位于c_k左侧的聚类块 c_i 与 c_k 的距离 d(中心距离或边缘距离)
 *        如果 d < W 则把 c_i 添加到 block_indexs
 *
 * 聚合位姿估算
 * ==========
 *    聚合点云 cloud_gather
 *    计算点云中心 (x, y, z)
 *    主成份分析(PCA)获取点云姿态 (R, P, Y)
 *    点云投影获取长宽高(L, W, H)
 *    合成位姿 (x, y, z, R, P, Y, L, W, H)
 *
 * 统计量: 匹配度计算
 * ================
 *    模板: (L, W, H)
 *    目标: (L', W', H')
 *    长宽高对齐
 *    计算 IoU
 *
 * 聚合评分
 * ==========
 *    聚合信息序列 gather_infos
 *    点数权重 w_points = 0.1
 *    距离权重 w_distance = 0.1
 *    匹配度权重 w_matching = 0.8
 *
 *    遍历聚合信息序列 gather_infos, 统计计量信息
 *      计算点数 points
 *      计算距离 distance
 *      计算匹配度 matching
 *
 *    遍历聚合信息序列 gather_infos, 计量信息归一化处理
 *
 *    遍历聚合信息序列 gather_infos
 *      聚合评分: score = w_points * points_norm + w_distance * distance_norm + w_matching * matching_norm
 *
 *
 * 算法流程
 * =======
 *  聚类块状态序列 vector<culster_stat> cluster_stats
 *  聚合信息序列   vector<gather_info>  gather_infos
 *
 *  遍历 cluster_indices，统计聚类块状态 cluster_stats
 *    计算聚类块边界（min_x, min_y, min_z, max_x, max_y, max_z）
 *
 *  遍历 cluster_indices: c_k
 *    以 c_k 为参考，聚合聚类块：
 *      聚合方法1 => c_k_m1 => push gather_infos
 *      聚合方法2 => c_k_m2 => push gather_infos
 *      聚合方法3 => c_k_m3 => push gather_infos
 *      聚合方法4 => c_k_m4 => push gather_infos
 *      聚合方法5 => c_k_m5 => push gather_infos
 *
 *  遍历 gather_infos
 *    聚合点云位姿估算
 *
 *  遍历 gather_infos
 *    聚合点云统计量计算(points, distance, matching)
 *
 *  遍历 gather_infos
 *    聚合点云统计量归一化(points_norm, distance_norm, matching_norm)
 *
 *  遍历 gather_infos
 *    聚合点云评分(score)
 *
 *  选择得分最高者输出
 *
 */

#include "common.h"

#include <iostream>
#include <algorithm>
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
#include <pcl/common/pca.h>


class ObjCloudPattern
{
public: // 类型定义
  // 聚类块状态
  typedef struct _ClusterState
  {
    int cluster_index;  // 聚类索引
    float x_min, y_min, z_min, x_max, y_max, z_max; // 聚类边界
  }ClusterState;

  // 聚合点云统计信息
  typedef struct _GatherInfo
  {
    std::set<int> block_indexs;  // 聚类块索引集合
    float bbox3d[9];                // 聚合点云位姿估算：（x,y,z,R,P,Y,L,W,H）
    int points;                     // 计量信息: 聚合点云点数量
    float distance;                 // 计量信息: 距离
    float matching;                 // 计量信息: 匹配度
    float points_norm;              // 计量信息归一化: 聚合点云点数量
    float distance_norm;            // 计量信息归一化: 距离
    float matching_norm;            // 计量信息归一化: 匹配度
    float score;                    // 聚合评分
  }GatherInfo;

public:
  ObjCloudPattern();
  ~ObjCloudPattern();

public:
  void setVerbose(int verbose);                                             // 设置调试等级
  //  void setClusters(int obj_index,
  //                   pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
  //                   const std::vector<pcl::PointIndices> &cluster_indices);  // 设置点云和聚类结果
  void setClassInfo(float L, float W, float H);                             // 设置类别配置信息
  void setWeights(float w_matching, float w_distance, float w_points);      // 设置权重信息
  void setGatherCoeff(float coeff);                                         // 聚合阈值系数
  void setPoseType(int n_type);                                             // 设置位姿输出方法
  void setDistanceType(int n_type);                                         // 设置距离计算方法
  void setNormType(int norm_type);                                          // 设置归一化方法

public:
  void apply(int obj_index,
             const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
             const std::vector<pcl::PointIndices> &cluster_indices,
             Eigen::Matrix<float, 1, 9> &pose);   // 目标区域匹配

private:
  float distance(const ClusterState &state_k, const ClusterState &state_i, int n_type=0) ; // 计算两个聚类块距离

  // 聚类块中心
  inline void get_center(const ClusterState &state, float &x, float &y, float &z)
  {
    x = (state.x_max + state.x_min)/2;
    y = (state.y_max + state.y_min)/2;
    z = (state.z_max + state.z_min)/2;
  }

private:
  void initClusterState(ClusterState &state);   // 初始化聚类块结构体
  void initGatherInfo(GatherInfo &info);        // 初始化聚合信息结构体
  void clusterStates(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                     const std::vector<pcl::PointIndices> &cluster_indices,
                     std::vector<ClusterState> &cluster_states);  // 统计聚类块信息
  void gatherByRadius(const std::vector<ClusterState> &cluster_states,
                      unsigned int c_k, float thr_r, int distance_type,
                      std::vector<GatherInfo> &gather_infos);     // 聚合方法1(半径聚合法)
  void gatherBySide(const std::vector<ClusterState> &cluster_states,
                    unsigned int c_k, float thr_w, int distance_type,
                    std::vector<GatherInfo> &gather_infos);       // 聚合方法2(偏旁聚合法)
  void clearRedundancy(std::vector<GatherInfo> &gather_infos);    // 清除聚合冗余
  void estimatePose(int obj_index,
                    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                    const std::vector<pcl::PointIndices> &cluster_indices,
                    float L, float W, float H,
                    std::vector<GatherInfo> &gather_infos);           // 聚合位姿估算
  float calculateMatching(float L, float W, float H, float Lk, float Wk, float Hk); // 匹配度计算
  void statisticNormalization(std::vector<GatherInfo> &gather_infos); // 统计量归一化
  void scores(float w_matching, float w_distance, float w_points,
              std::vector<GatherInfo> &gather_infos);                 // 聚合评分
  void getMaxScore(const std::vector<ClusterState> &states,
                   const std::vector<GatherInfo> &gather_infos, int pose_type,
                   Eigen::Matrix<float, 1, 9> &pose);   // 返回评分最高者
  void getGatherCloudBoundary(const std::vector<ClusterState> &states, const GatherInfo &info,
                              float &x_min, float &y_min, float &z_min,
                              float &x_max, float &y_max, float &z_max);// 获取聚合点云边界
  void sort_data(float *data, int *indexs, int N);      // LWH排序: 降序

  void print_cluster_state(int index, const ClusterState &state);     // 打印聚类块信息
  void print_gather_info(int index, const GatherInfo &info);          // 打印聚合信息
  void print_cluster_states(const std::vector<ClusterState> &states); // 打印聚类块信息
  void print_gather_infos(const std::vector<GatherInfo> &infos);      // 打印聚合信息

private:
  int verbose_;                                           // 调试等级
  //  int obj_index_;                                         // 目标序号
  //  pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_;       // 输入点云
  //  const std::vector<pcl::PointIndices> *cluster_indices_; // 聚类序列
  std::vector<ClusterState> cluster_states_;              // 聚类状态序列
  std::vector<GatherInfo> gather_infos_;                  // 聚合点云统计信息序列
  float L_, W_, H_;                                       // 目标模板尺寸: 长宽高(m)
  float w_points_, w_distance_, w_matching_;              // 权重(点数， 距离， 匹配度)
  float gather_coeff_ ; // 聚合阈值系数 gather_coeff,用于调整R,D值, R用于半径聚合， D用于偏旁聚合
                        //  *         |<--R-->|
                        //  * |-------|xxxxxxx|---o---|xxxxxxx|-------|
                        //  *
                        //  *             |<-------D------>|
                        //  * |-----------|xxxxxxxxxxxxxxxx|----------|
                        //
                        //    R=max(L, W, H) * 1/4 * coeff, D=max(L, W, H) * 1/2 *coeff
  int pose_type_;       // 位姿输出方法, 0-由PCA估算位姿, 1-由聚合点云边界估算（忽略姿态信息）
  int distance_type_;   // 距离计算方法: 0-中心距离，1-边缘距离
  int norm_type_;       // 统计量正则化方法:
                        //    0-最大值归一化，最大值项正则值为1， 其他项正则值为与最大值的比值
                        //    1-总数归一化, 累积所有项值的和， 每一项的正则值为项值与总和的比值
};


ObjCloudPattern::ObjCloudPattern()
{
  w_matching_ = 0.4;
  w_distance_ = 0.3;
  w_points_   = 0.3;

  gather_coeff_ = 1.0;
  pose_type_ = 0;
  distance_type_ = 1;
}

ObjCloudPattern::~ObjCloudPattern()
{

}

// 设置调试等级
void ObjCloudPattern::setVerbose(int verbose)
{
  verbose_ = verbose;
}

///** @brief 设置点云和聚类结果
// * @param obj_index in - 目标序号
// * @param cloud in - 输入点云
// * @param cluster_indices in - 聚类结果
// */
//void ObjCloudPattern::setClusters(
//    int obj_index,
//    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
//    const std::vector<pcl::PointIndices> &cluster_indices)
//{
//  obj_index_ = obj_index;
//  cloud_ = cloud;
//  cluster_indices_ = &cluster_indices;
//  if (verbose_>1) {
//    printf("[I:%s:%d][ObjCloudPattern::setClusters] obj_index=%d, all points=%ld, clusters=%ld\n", __FILE__, __LINE__,
//           obj_index, cloud->points.size(), cluster_indices.size());
//  }
//}

/** @brief 设置类别配置信息
 * @param L in - 长(m)
 * @param W in - 宽(m)
 * @param H in - 高(m)
 */
void ObjCloudPattern::setClassInfo(float L, float W, float H)
{
  L_ = L;
  W_ = W;
  H_ = H;
}

/** @brief 设置权重信息
 * @param w_matching in - 匹配度权重
 * @param w_distance in - 距离权重
 * @param w_points   in - 点数量权重
 */
void ObjCloudPattern::setWeights(float w_matching, float w_distance, float w_points)
{
  w_matching_ = w_matching;
  w_distance_ = w_distance;
  w_points_   = w_points;
}

/** @brief 设置聚合阈值系数
 * @param coeff in - 聚合阈值系数
 */
void ObjCloudPattern::setGatherCoeff(float coeff)
{
  gather_coeff_ = coeff;
}

// 设置位姿输出方法
void ObjCloudPattern::setPoseType(int n_type)
{
  pose_type_ = n_type;
}

// 设置距离计算方法
void ObjCloudPattern::setDistanceType(int n_type)
{
  distance_type_ = n_type;
}

// 设置归一化方法
void ObjCloudPattern::setNormType(int norm_type)
{
  norm_type_ = norm_type;
}

/** @brief 目标区域匹配
 * 算法流程
 * =======
 *  聚类块状态序列 vector<culster_stat> cluster_stats
 *  聚合信息序列   vector<gather_info>  gather_infos
 *
 *  遍历 cluster_indices，统计聚类块状态 cluster_stats
 *    计算聚类块边界（min_x, min_y, min_z, max_x, max_y, max_z）
 *
 *  遍历 cluster_indices: c_k
 *    以 c_k 为参考，聚合聚类块：
 *      聚合方法1 => c_k_m1 => push gather_infos
 *      聚合方法2 => c_k_m2 => push gather_infos
 *
 *  清除聚合冗余信息
 *
 *  遍历 gather_infos
 *    聚合点云位姿估算
 *
 *  遍历 gather_infos
 *    聚合点云统计量计算(points, distance, matching)
 *
 *  遍历 gather_infos
 *    聚合点云统计量归一化(points_norm, distance_norm, matching_norm)
 *
 *  遍历 gather_infos
 *    聚合点云评分(score)
 *
 *  选择得分最高者输出
 *
 * @param obj_index in - 目标序号
 * @param cloud in - 目标2D框内点云
 * @param cluster_indices in - 聚类结果
 * @param pose out - 返回目标位姿描述, (x, y, z, R, P, Y, L, W, H)
 */
void ObjCloudPattern::apply(int obj_index,
           const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
           const std::vector<pcl::PointIndices> &cluster_indices,
           Eigen::Matrix<float, 1, 9> &pose)
{
  // 清空上次结果
  cluster_states_.clear();
  gather_infos_.clear();

  if (cluster_indices.size()<=0) {
    printf("[E:%s:%d][ObjCloudPattern::apply] cluster_indices is empty!\n", __FILE__, __LINE__);
    return;
  }

  if (verbose_>0) {
    printf("\n---------- ObjCloudPattern::apply -----------\n");
    printf("[I:%s:%d][ObjCloudPattern::apply] obj_index=%d, clusters=%ld\n", __FILE__, __LINE__,
           obj_index, cluster_indices.size());
  }
  float thr_max = L_ > W_ ? (L_ > H_ ? L_ : H_) : (W_ > H_ ? W_ : H_); // max(L, W, H)
  float thr_R = thr_max * gather_coeff_ / 4;  // 半径聚合基准阈值: 应用阈值需要再乘于系数gather_coeff
  float thr_D = thr_max * gather_coeff_ / 2;  // 偏旁聚合基准阈值: 应用阈值需要再乘于系数gather_coeff
  if (verbose_>1) {
    printf("[I:%s:%d][ObjCloudPattern::apply] parameters:\n"
           "    L=%f, W=%f, H=%f, w_points=%f, w_distance=%f, w_matching=%f, gather_coeff=%f, thr_max=%f, thr_R=%f, thr_D=%f\n",
           __FILE__, __LINE__,
           L_, W_, H_, w_points_, w_distance_, w_matching_, gather_coeff_, thr_max, thr_R, thr_D);
    printf("    Has %ld clusters in obj_cound\n", cluster_indices.size());
  }
  if (verbose_>1) {
    printf("---------- clusterStates -----------\n");
  }
  clusterStates(cloud, cluster_indices, cluster_states_);  // 统计聚类块信息
  if (verbose_>1) {
    printf("[I:%s:%d][ObjCloudPattern::apply] After cuslterStates(...)\n", __FILE__, __LINE__);
    print_cluster_states(cluster_states_);
  }
  if (verbose_>1) {
    printf("---------- gatherInfos -----------\n");
  }
  for(unsigned int c_k = 0; c_k < cluster_states_.size(); c_k++) {
    gatherByRadius(cluster_states_, c_k, thr_R, distance_type_, gather_infos_);     // 聚合(半径)
    gatherBySide(cluster_states_, c_k, thr_D, distance_type_, gather_infos_);       // 聚合(偏旁)
  }
  if (verbose_>1) {
    printf("[I:%s:%d][ObjCloudPattern::apply] After gatherByRadius and gatherBySide ...\n", __FILE__, __LINE__);
    print_gather_infos(gather_infos_);
  }
  if (verbose_>1) {
    printf("---------- clearRedundancy -----------\n");
  }
  clearRedundancy(gather_infos_); // 清除聚合冗余
  if (verbose_>1) {
    printf("[I:%s:%d][ObjCloudPattern::apply] After clearRedundancy ...\n", __FILE__, __LINE__);
    print_gather_infos(gather_infos_);
  }
  if (verbose_>1) {
    printf("---------- estimatePose -----------\n");
  }
  estimatePose(obj_index, cloud, cluster_indices, L_, W_, H_, gather_infos_); //位姿估算, 统计量计算
  if (verbose_>1) {
    printf("[I:%s:%d][ObjCloudPattern::apply] After estimatePose ...\n", __FILE__, __LINE__);
    print_gather_infos(gather_infos_);
  }
  if (verbose_>1) {
    printf("---------- statisticNormalization -----------\n");
  }
  statisticNormalization(gather_infos_);                               // 统计量归一化
  if (verbose_>1) {
    printf("[I:%s:%d][ObjCloudPattern::apply] After statisticNormalization ...\n", __FILE__, __LINE__);
    print_gather_infos(gather_infos_);
  }
  if (verbose_>1) {
    printf("---------- scores -----------\n");
  }
  scores(w_matching_, w_distance_, w_points_, gather_infos_);            // 聚合评分
  if (verbose_>1) {
    printf("[I:%s:%d][ObjCloudPattern::apply] After scores ...\n", __FILE__, __LINE__);
    print_gather_infos(gather_infos_);
  }
  if (verbose_>1) {
    printf("---------- getMaxScore -----------\n");
  }
  getMaxScore(cluster_states_, gather_infos_, pose_type_, pose); // 返回评分最高者
  if (verbose_>1) {
    printf("[I:%s:%d][ObjCloudPattern::apply] getMaxScore \n", __FILE__, __LINE__);
    std::cout << pose << std::endl;
  }
}

/** @brief 计算两个聚类块距离
 *
 *  中心距离
 *  =======
 *    .   .         .   .
 *  .   o<--+--d--+-->o---.
 *    .   .         .   .
 *
 * 边缘距离：世界坐标系，忽略z轴
 * =========================
 *    .   .          .   .
 *  .   o   .<--d-->.  o  .
 *    .   .          .   .
 *
 * 边缘距离简化形式
 * ==============
 *              x
 *   +----------^---------+
 *   | [1]     [8]    [7] |
 *   |          |         |
 *   |          |         |
 * y <-[2]-----c_k----[6]-+
 *   |          |         |
 *   |          |         |
 *   | [3]     [4]    [5] |
 *   +----------+---------+
 *
 * [2,6]
 * +------+
 * |      |       +-------+
 * |      |<--d-->|       |
 * +------+       |       |
 *                +-------+
 * [4,8]
 * +------+
 * |      |
 * |      |
 * +------+
 *      ^
 *      |
 *      d
 *      |
 *      v
 *    +-------+
 *    |       |
 *    |       |
 *    +-------+
 *
 * [1,3,5,7]
 * +------+
 * |      |
 * |      |
 * +------+
 *          .
 *            d
 *              .
 *                +-------+
 *                |       |
 *                |       |
 *                +-------+
 *
 * @param state_k in - 参考聚类块
 * @param state_i in - 邻近聚类块
 * @param n_type  in - 计算方法，0-中心距离， 1-边缘距离
 */
float ObjCloudPattern::distance(const ClusterState &state_k, const ClusterState &state_i, int n_type)
{
  float d = 0;
  switch(n_type)
  {
  case 1:
  {
    if ((state_i.x_min >= state_k.x_max) && (state_i.y_min >= state_k.y_max))
    {
      // [1]
      d = sqrtf(powf(state_i.x_min-state_k.x_max, 2.0) + powf(state_i.y_min-state_k.y_max, 2.0));
    }
    else if((state_i.y_min >= state_k.y_max) &&
              ((state_i.x_min <= state_k.x_max && state_i.x_min >= state_k.x_min) ||
               (state_i.x_max <= state_k.x_max && state_i.x_max >= state_k.x_min)))
    {
      // [2]
      d = state_i.y_min - state_k.y_max;
    }
    else if ((state_i.x_max <= state_k.x_min) && (state_i.y_min >= state_k.y_max))
    {
      // [3]
      d = sqrtf(powf(state_k.x_min-state_i.x_max, 2.0) + powf(state_i.y_min-state_k.y_max, 2.0));
    }
    else if((state_i.x_max <= state_k.x_min) &&
              ((state_i.y_min <= state_k.y_max && state_i.y_min >= state_k.y_min) ||
               (state_i.y_max <= state_k.y_max && state_i.y_max >= state_k.y_min)))
    {
      // [4]
      d = state_k.x_min - state_i.x_max;
    }
    else if((state_i.x_max <= state_k.x_min) && (state_i.y_max <= state_k.y_min))
    {
      // [5]
      d = sqrtf(powf(state_k.x_min-state_i.x_max, 2.0) + powf(state_k.y_min-state_i.y_max, 2.0));
    }
    else if((state_i.y_max <= state_k.y_min) &&
              ((state_i.x_min <= state_k.x_max && state_i.x_min >= state_k.x_min) ||
               (state_i.x_max <= state_k.x_max && state_i.x_max >= state_k.x_min)))
    {
      // [6]
      d = state_k.y_min - state_i.y_max;
    }
    else if ((state_i.x_min >= state_k.x_max) && (state_i.y_max <= state_k.y_min))
    {
      // [7]
      d = sqrtf(powf(state_i.x_min-state_k.x_max, 2.0) + powf(state_k.y_min-state_i.y_max, 2.0));
    }
    else if((state_i.x_min >= state_k.x_max) &&
            ((state_i.y_min <= state_k.y_max && state_i.y_min >= state_k.y_min) ||
             (state_i.y_max <= state_k.y_max && state_i.y_max >= state_k.y_min)))
    {
      // [8]
      d = state_i.x_min - state_k.x_max;
      break;
    }
    else
    {
      d = sqrtf(
              powf((state_k.x_max+state_k.x_min)/2 - (state_i.x_max+state_i.x_min)/2, 2.0) +
              powf((state_k.y_max+state_k.y_min)/2 - (state_i.y_max+state_i.y_min)/2, 2.0) +
              powf((state_k.z_max+state_k.z_min)/2 - (state_i.z_max+state_i.z_min)/2, 2.0));
    }
  } /* end case 1*/
  default:
  {
    d = sqrtf(
            powf((state_k.x_max+state_k.x_min)/2 - (state_i.x_max+state_i.x_min)/2, 2.0) +
            powf((state_k.y_max+state_k.y_min)/2 - (state_i.y_max+state_i.y_min)/2, 2.0) +
            powf((state_k.z_max+state_k.z_min)/2 - (state_i.z_max+state_i.z_min)/2, 2.0));
    break;
  }
  }

  return d;
}

/** @brief 初始化聚类块结构体
 * @param state in out - 聚类块
 */
void ObjCloudPattern::initClusterState(ClusterState &state)
{
  state.cluster_index = -1;
  state.x_min = 0;
  state.y_min = 0;
  state.z_min = 0;
  state.x_max = 0;
  state.y_max = 0;
  state.z_max = 0;
}

/** @brief 初始化聚合信息结构体
 * @param info in out - 聚合信息
 */
void ObjCloudPattern::initGatherInfo(GatherInfo &info)
{
  for(int i=0; i<9; i++) { info.bbox3d[i]=0; }
  info.distance = 0;
  info.distance_norm = 0;
  info.matching = 0;
  info.matching_norm = 0;
  info.points = 0;
  info.matching_norm = 0;
  info.score = 0;
}

/** @brief 统计聚类块信息
 * @param cloud in - 输入点云
 * @param cluster_indices in - 聚类结果
 * @param cluster_states out - 统计信息
 */
void ObjCloudPattern::clusterStates(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                   const std::vector<pcl::PointIndices> &cluster_indices,
                   std::vector<ClusterState> &cluster_states)
{
  cluster_states.clear();
  for(unsigned int i = 0; i<cluster_indices.size(); i++) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Vector4f min_val, max_val, mean_val;
    pcl::PointIndices::Ptr indices(new pcl::PointIndices(cluster_indices[i]));
    CommonFuns::extract_points(cloud, indices, cloud_out); // 提取聚类块点云
    CommonFuns::points_stat(cloud_out, min_val, max_val, mean_val);   // 点云统计获取边界

    ClusterState state;
    state.cluster_index = i;
    state.x_min = min_val[0];
    state.y_min = min_val[1];
    state.z_min = min_val[2];
    state.x_max = max_val[0];
    state.y_max = max_val[1];
    state.z_max = max_val[2];
    cluster_states.push_back(state);
  }
}

/** @brief 聚合方法1(半径聚合法)
 *
 * 模型
 * ====
 *    obj
 *    |<-------R--------o---------------->|
 *      c_i            c_k           c_j
 *    |------|xxxxxx|-------|xxxxx|-------|
 *
 * 算法流程
 * =======
 *    聚合半径 R = max(L, W, H)*2/3
 *    参考聚类块 c_k
 *    聚合子列 block_indexs
 *    遍历聚类块状态序列 cluster_stats
 *      计算聚类块 c_i 与 c_k 的距离 d(中心距离或边缘距离)
 *      如果 d < R 则把 c_i 添加到 block_indexs
 *
 * @param cluster_states in - 聚类块统计信息序列
 * @param c_k in - 参考聚类块
 * @param thr_r in - 聚合半径阈值
 * @param distance_type in - 距离计算方法: 0-中心距离， 1-边缘距离
 * @param gather_infos out - 聚合信息序列
 */
void ObjCloudPattern::gatherByRadius(
    const std::vector<ClusterState> &cluster_states,
    unsigned int c_k, float thr_r, int distance_type,
    std::vector<GatherInfo> &gather_infos)

{
  GatherInfo info;
  initGatherInfo(info);

  for(unsigned int i=0; i<cluster_states.size(); i++) {
    float d = distance(cluster_states[c_k], cluster_states[i], distance_type);
    if (verbose_>2) {
      printf("[ObjCloudPattern::gatherByRadius] distance(c_k:%d, i:%d, dis_type:1)=%f, thr_r=%f\n", c_k, i, d, thr_r);
    }
    if (c_k !=i && d > thr_r) {continue;} // 大于阈值跳过
    if (verbose_>2) {
      printf("[ObjCloudPattern::gatherByRadius] gather -> distance(c_k:%d, i:%d, dis_type:1)=%f, thr_r=%f\n", c_k, i, d, thr_r);
    }
    info.block_indexs.insert(i); // 添加聚类块
  }

  gather_infos.push_back(info); // 添加到聚合序列
}

/** @brief 聚合方法2(偏旁聚合法)
 * 模型
 * ====
 *                     i
 *               o     ^    o
 *            o        |       o
 *          o     [1] thr_w  [4] o
 *                  .  |   .       o
 *         o-------.---k----.-------o
 *          o       .  |  .        o
 *                [2]  .     [3]  o
 *              o      |      o
 *                     o
 *
 * 算法流程
 * =======
 *    聚合偏旁宽度 W = max(L, W, H)
 *    参考聚类块 c_k
 *    划分四个区域: [1],[2],[3],[4]
 *      聚合子列 block_indexs
 *      遍历聚类块状态序列 cluster_stats
 *        计算位于c_k左侧的聚类块 c_i 与 c_k 的距离 d
 *        如果 d < W 则把 c_i 添加到 block_indexs
 *
 * @param cluster_states in - 聚类块统计信息序列
 * @param c_k in - 参考聚类块
 * @param thr_w in - 聚合宽度阈值
 * @param distance_type in - 距离计算方法: 0-中心距离， 1-边缘距离
 * @param gather_infos out - 聚合信息序列
 */
void ObjCloudPattern::gatherBySide(
    const std::vector<ClusterState> &cluster_states,
    unsigned int c_k, float thr_w, int distance_type,
    std::vector<GatherInfo> &gather_infos)
{
  // 获取参考聚类块中心坐标
  float ck_center_x=0, ck_center_y=0, ck_center_z=0;
  get_center(cluster_states[c_k], ck_center_x, ck_center_y, ck_center_z);

  // 初始化聚合信息: 四个区域互相独立
  GatherInfo infos[4];
  for(int i=0; i<4; i++)
  {
    initGatherInfo(infos[i]);
  }

  // 遍历聚类块序列，进行聚合搜索
  for(unsigned int i=0; i<cluster_states.size(); i++) {
    if (c_k==i) {continue;}                                                   // 参考聚类块跳过
    float d = distance(cluster_states[c_k], cluster_states[i], distance_type);
    if (verbose_>2) {
      printf("[ObjCloudPattern::gatherBySide] distance(c_k:%d, i:%d, dis_type:1)=%f, thr_r=%f\n", c_k, i, d, thr_w);
    }
    if (d > thr_w) {continue;} // 距离大于阈值跳过

    if (verbose_>2) {
      printf("[ObjCloudPattern::gatherBySide] gather distance(c_k:%d, i:%d, dis_type:1)=%f, thr_r=%f\n", c_k, i, d, thr_w);
    }

    // 获取聚类块中心坐标
    float ci_center_x=0, ci_center_y=0, ci_center_z=0;
    get_center(cluster_states[i], ci_center_x, ci_center_y, ci_center_z);

    // 四个区区域独立处理
    if ((ci_center_x > ck_center_x) && (ci_center_y > ck_center_y)) {
      /* 0: x>0, y>0 */
      if (infos[0].block_indexs.size()<=0) {
        // 首次添加参考聚类块, 下同
        infos[0].block_indexs.insert(c_k);
      }
      if (verbose_>2) {
        printf("[ObjCloudPattern::gatherBySide][0] ck_x=%f, ck_y=%f, ci_x=%f, ci_y=%f\n",
               ck_center_x, ck_center_y, ci_center_x, ci_center_y);
      }
      infos[0].block_indexs.insert(i);
    } else if ((ci_center_x < ck_center_x) && (ci_center_y > ck_center_y)) {
      /* 1: x<0, y>0 */
      if (infos[1].block_indexs.size()<=0) {
        infos[1].block_indexs.insert(c_k);
      }
      if (verbose_>2) {
        printf("[ObjCloudPattern::gatherBySide][1] ck_x=%f, ck_y=%f, ci_x=%f, ci_y=%f\n",
               ck_center_x, ck_center_y, ci_center_x, ci_center_y);
      }
      infos[1].block_indexs.insert(i);
    } else if ((ci_center_x < ck_center_x) && (ci_center_y < ck_center_y)) {
      /* 2: x<0, y<0 */
      if (infos[2].block_indexs.size()<=0) {
        infos[2].block_indexs.insert(c_k);
      }
      if (verbose_>2) {
        printf("[ObjCloudPattern::gatherBySide][2] ck_x=%f, ck_y=%f, ci_x=%f, ci_y=%f\n",
               ck_center_x, ck_center_y, ci_center_x, ci_center_y);
      }
      infos[2].block_indexs.insert(i);
    } else if ((ci_center_x > ck_center_x) && (ci_center_y < ck_center_y)) {
      /* 3: x>0, y<0 */
      if (infos[3].block_indexs.size()<=0) {
        infos[3].block_indexs.insert(c_k);
      }
      if (verbose_>2) {
        printf("[ObjCloudPattern::gatherBySide][3] ck_x=%f, ck_y=%f, ci_x=%f, ci_y=%f\n",
               ck_center_x, ck_center_y, ci_center_x, ci_center_y);
      }
      infos[3].block_indexs.insert(i);
    }
  }

  // 四个区域分别加入聚合序列
  for(int i=0; i<4; i++) {
    if (infos[i].block_indexs.size()>0) {
      gather_infos.push_back(infos[i]);
    }
  }
}

// 清除聚合冗余
void ObjCloudPattern::clearRedundancy(std::vector<GatherInfo> &gather_infos)
{
  if (gather_infos.size()<2) {
    return;
  }

  std::vector<GatherInfo> ret;
  ret.push_back(gather_infos[0]);
  for(unsigned int i = 1; i<gather_infos.size(); i++)
  {
    bool b_exist = false;
    for(unsigned int j=0; j<ret.size(); j++)
    {
      if (std::equal(gather_infos[i].block_indexs.begin(), gather_infos[i].block_indexs.end(),
                     ret[j].block_indexs.begin(), ret[j].block_indexs.end()))
      {
        b_exist = true;
      }
    }
    if (!b_exist)
    {
      ret.push_back(gather_infos[i]);
    }
  }
  gather_infos = ret;
}

/** @brief 聚合位姿估算
 *
 * 算法流程
 * =======
 *    聚合点云 cloud_gather
 *    计算点云中心 (x, y, z)
 *    主成份分析(PCA)获取点云姿态 (R, P, Y)
 *    点云投影获取长宽高(L, W, H)
 *    合成位姿 (x, y, z, R, P, Y, L, W, H)
 *
 * @param obj_index in - 目标序号
 * @param cloud in - 输入点云
 * @param cluster_indices in - 聚类结果
 * @param L, W, H in - 目标模板尺寸: 长宽高(m)
 * @param gather_infos out - 聚合信息
 */
void ObjCloudPattern::estimatePose(
    int obj_index,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    const std::vector<pcl::PointIndices> &cluster_indices,
    float L, float W, float H,
    std::vector<GatherInfo> &gather_infos)
{
  if (verbose_>1) {
    printf("[I:%s:%d][ObjCloudPattern::estimatePose] obj_index=%d, gathers=%ld\n", __FILE__, __LINE__,
           obj_index, gather_infos.size());
  }
  Eigen::Vector4f min_val, max_val, mean_val;

  for (unsigned int i=0; i<gather_infos.size(); i++) {
    if (verbose_>1) {
      printf("[I:%s:%d][ObjCloudPattern::estimatePose] obj_index=%d, gather_index=%d\n", __FILE__, __LINE__,
             obj_index, i);
      if (verbose_>2) {
        print_gather_info(i, gather_infos[i]);
      }
    }

    // 聚合点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gather(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    std::set<int>::iterator it_block_index;
    for (it_block_index=gather_infos[i].block_indexs.begin();
         it_block_index!=gather_infos[i].block_indexs.end(); it_block_index++)
    {
      // printf("[D:%s:%d] clusters=%ld, it_block_index=%d, points=%ld\n", __FILE__, __LINE__,
      //        cluster_indices.size(), *it_block_index, cluster_indices[*it_block_index].indices.size());
      for(unsigned int k=0; k<cluster_indices[*it_block_index].indices.size(); k++)
      {
        indices->indices.push_back(cluster_indices[*it_block_index].indices[k]);
      }
    }

    CommonFuns::extract_points(cloud, indices, cloud_gather);

    if (verbose_>2) {
      char pcd_file[255] = {0};
      sprintf(pcd_file, "/tmp/obj_cloud_gather_%d_%d.pcd", obj_index, i);
      pcl::io::savePCDFile(pcd_file, *cloud_gather);
      printf("[D:%s:%d][ObjCloudPattern::estimatePose] save to '%s'\n", __FILE__, __LINE__, pcd_file);
    }

    // PCA分析
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(cloud_gather);
    Eigen::Vector3f eigen_values = pca.getEigenValues();  // 特征值;
    Eigen::Matrix3f eigen_vector = pca.getEigenVectors(); // 特征向量;
    Eigen::Vector4f eigen_means  = pca.getMean();         // 中心坐标

    if (verbose_>2) {
      std::cout << "[ObjCloudPattern::estimatePose] eigen_values: " << eigen_values << std::endl;
      std::cout << "[ObjCloudPattern::estimatePose] eigen_vector: " << std::endl << eigen_vector << std::endl;
      std::cout << "[ObjCloudPattern::estimatePose] eigen_means : " << eigen_means << std::endl;
    }

    // 变换矩阵
    Eigen::Matrix4f T;
    T.setIdentity();
    T.block(0, 0, 3, 3) = eigen_vector;
    T.block(0, 3, 3, 1) = eigen_means.block(0, 0, 3, 1);

    // 位姿
    Eigen::Matrix<float, 1, 6> pose = CommonFuns::T2pose(T);
    if (verbose_>2) {
      std::cout << "[ObjCloudPattern::estimatePose] T: " << std::endl << T << std::endl;
      std::cout << "[ObjCloudPattern::estimatePose] pose: " << pose << std::endl;
    }

    // 正交投影: 将点或者点云投影到特征向量（正交）建立的局部坐标系下
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_project(new pcl::PointCloud<pcl::PointXYZI>());
    pca.project(*cloud_gather, *cloud_project);
    if (verbose_>2) {
      char pcd_file[255] = {0};
      sprintf(pcd_file, "/tmp/obj_cloud_gather_%d_%d_project.pcd", obj_index, i);
      pcl::io::savePCDFile(pcd_file, *cloud_project);
    }

    // 获取投影点云长宽高
    CommonFuns::points_stat(cloud_project, min_val, max_val, mean_val);
    float Lc = max_val[0] - min_val[0];
    float Wc= max_val[1] - min_val[1];
    float Hc = max_val[2] - min_val[2];

    if(verbose_>2) {
      printf("[ObjCloudPattern::estimatePose] cloud_project: points=%ld, x_min=%f, y_min=%f, z_min=%f, x_max=%f, y_max=%f, z_max=%f\n"
             "L=%f, W=%f, H=%f\n", cloud_project->points.size(), min_val[0], min_val[1], min_val[2], max_val[0], max_val[1], max_val[2],
             max_val[0]-min_val[0], max_val[1]-min_val[1], max_val[2]-min_val[2]);
      CommonFuns::points_stat(cloud_gather, min_val, max_val, mean_val);
      printf("[ObjCloudPattern::estimatePose] cloud_gather: points=%ld, x_min=%f, y_min=%f, z_min=%f, x_max=%f, y_max=%f, z_max=%f\n"
             "L=%f, W=%f, H=%f\n", cloud_gather->points.size(), min_val[0], min_val[1], min_val[2], max_val[0], max_val[1], max_val[2],
             max_val[0]-min_val[0], max_val[1]-min_val[1], max_val[2]-min_val[2]);
    }

    // 位姿合成
    gather_infos[i].bbox3d[0] = pose[0]; // x
    gather_infos[i].bbox3d[1] = pose[1]; // y
    gather_infos[i].bbox3d[2] = pose[2]; // z
    gather_infos[i].bbox3d[3] = pose[3]; // R
    gather_infos[i].bbox3d[4] = pose[4]; // P
    gather_infos[i].bbox3d[5] = pose[5]; // Y
    gather_infos[i].bbox3d[6] = Lc;       // L
    gather_infos[i].bbox3d[7] = Wc;       // W
    gather_infos[i].bbox3d[8] = Hc;       // H

    // 聚合点云统计量
    gather_infos[i].points = cloud_gather->points.size(); // 点云点数量
    gather_infos[i].distance = sqrtf(powf(gather_infos[i].bbox3d[0], 2.0) +
                                     powf(gather_infos[i].bbox3d[1], 2.0) +
                                     powf(gather_infos[i].bbox3d[2], 2.0)); // 距离
    gather_infos[i].matching = calculateMatching(L, W, H, Lc, Wc, Hc); // 匹配度

    if (verbose_>2) {
      printf("[ObjCloudPattern::estimatePose] points=%d, distance=%f, matching=%f\n",
             gather_infos[i].points, gather_infos[i].distance, gather_infos[i].matching);
    }
  }
}

/** @brief 匹配度计算
 *    基于IoU概念计算匹配度
 * 算法流程
 * =======
 *  1. 由大到小重新排序(L, W, H)
 *  2. 由大到小重新排序(Lk, Wk, Hk)
 *  3. 取(L, W, H)与(Lk, Wk, Hk)最小值 (Lmin, Wmin, Hmin)
 *  4. 计算 (L, W, H) 体积 v
 *  5. 计算 (Lk, Wk, Hk) 体积 vk
 *  6. 计算 (Lmin, Wmin, Hmin) 体积 vmin
 *  7. 计算匹配度 2*vmin / (v+vk)
 *
 * @param L, W, H in - 模板长宽高
 * @param Lk, Wk, Hk in - 目标长宽高
 */
float ObjCloudPattern::calculateMatching(float L, float W, float H, float Lk, float Wk, float Hk)
{
  float LWH[3] = {L, W, H};
  float LWHk[3] = {Lk, Wk, Hk};
  float LWHmin[3] = {0, 0, 0};
  std::sort(LWH, LWH+3, std::greater<float>());
  std::sort(LWHk, LWHk+3, std::greater<float>());

  for (int i=0; i<3; i++) {
    LWHmin[i] = LWH[i] < LWHk[i] ? LWH[i] : LWHk[i];
  }

  float v    = LWH[0]    * LWH[1]    * LWH[2];
  float vk   = LWHk[0]   * LWHk[1]   * LWHk[2];
  float vmin = LWHmin[0] * LWHmin[1] * LWHmin[2];

  if (verbose_>2) {
    float matching = (2*vmin) / (v + vk);
    printf("[ObjCloudPattern::calculateMatching] L=%f, W=%f, H=%f, Lk=%f, Wk=%f, Hk=%f\n", L, W, H, Lk, Wk, Hk);
    printf("[ObjCloudPattern::calculateMatching] LWH(template    ):[%f, %f, %f]\n", LWH[0], LWH[1], LWH[2]);
    printf("[ObjCloudPattern::calculateMatching] LWH(real-project):[%f, %f, %f]\n", LWHk[0], LWHk[1], LWHk[2]);
    printf("[ObjCloudPattern::calculateMatching] LWHmin:[%f, %f, %f]\n", LWHmin[0], LWHmin[1], LWHmin[2]);
    printf("[ObjCloudPattern::calculateMatching] matching=%f, v=%f, vk=%f, vmin=%f\n", matching, v, vk, vmin);
  }

  return (2*vmin) / (v + vk);
}

/** @brief 统计量归一化
 * @param gather_infos in out - 聚合信息
 */
void ObjCloudPattern::statisticNormalization(std::vector<GatherInfo> &gather_infos)
{
  if (gather_infos.size()<=0) {
    return ;
  }

  switch(norm_type_)
  {
  case 0: // 最大值比值法
  {
    float min_points = 10000000000;
    float max_points = -1;
    float min_distance = 10000000000;
    float max_distance = -1;
    float min_matching = 10000000000;
    float max_matching = -1;

    // 统计最大最小值
    for (unsigned int i=0; i<gather_infos.size(); i++) {
      if (gather_infos[i].points<min_points) {
        min_points = gather_infos[i].points;
      }
      if (gather_infos[i].points>max_points) {
        max_points = gather_infos[i].points;
      }
      if (gather_infos[i].distance<min_distance) {
        min_distance = gather_infos[i].distance;
      }
      if (gather_infos[i].distance>max_distance) {
        max_distance = gather_infos[i].distance;
      }
      if (gather_infos[i].matching<min_matching) {
        min_matching = gather_infos[i].matching;
      }
      if (gather_infos[i].matching>max_matching) {
        max_matching = gather_infos[i].matching;
      }
    }

    // 归一化
    for (unsigned int i=0; i<gather_infos.size(); i++) {
      if (max_points>0) {
        // gather_infos[i].points_norm = (gather_infos[i].points - min_points) / (max_points-min_points);
        gather_infos[i].points_norm = gather_infos[i].points * 1.0 / max_points;
      }
      if (max_distance>0) {
        // gather_infos[i].distance_norm = 1 - (gather_infos[i].distance - min_distance) / (max_distance-min_distance);
        gather_infos[i].distance_norm = 1 / (gather_infos[i].distance / min_distance);
      }
      if (max_matching>0) {
        // gather_infos[i].matching_norm = (gather_infos[i].matching - min_matching) / (max_matching-min_matching);
        gather_infos[i].matching_norm = gather_infos[i].matching / max_matching;
      }
    }
    break;
  }
  case 1: // 总和比值法
  {
    float sum_points = 0;
    float sum_distance = 0;
    float sum_matching = 0;

    // 累积和
    for (unsigned int i=0; i<gather_infos.size(); i++) {
      // 累积和
      sum_points += gather_infos[i].points;
      sum_distance += gather_infos[i].distance;
      sum_matching += gather_infos[i].matching;
      // 初始化正则值
      gather_infos[i].points_norm = 0;
      gather_infos[i].distance_norm = 0;
      gather_infos[i].matching_norm =0;
    }

    // 归一化
    for (unsigned int i=0; i<gather_infos.size(); i++) {
      if (sum_points>0) {
        // gather_infos[i].points_norm = (gather_infos[i].points - min_points) / (max_points-min_points);
        gather_infos[i].points_norm = gather_infos[i].points * 1.0 / sum_points;
      }
      if (sum_distance>0) {
        // gather_infos[i].distance_norm = 1 - (gather_infos[i].distance - min_distance) / (max_distance-min_distance);
        gather_infos[i].distance_norm = 1 - gather_infos[i].distance / sum_distance;
      }
      if (sum_matching>0) {
        // gather_infos[i].matching_norm = (gather_infos[i].matching - min_matching) / (max_matching-min_matching);
        gather_infos[i].matching_norm = gather_infos[i].matching / sum_matching;
      }
    }
    break;
  }
  default:// 不支持
  {
    printf("[E:%s:%d][ObjCloudPattern::statisticNormalization] unsupport norm_type %d\n", __FILE__, __LINE__, norm_type_);
    break;
  }
  }
}

/** @brief 聚合评分
 * 算法流程
 * =======
 *    聚合信息序列 gather_infos
 *    点数权重 w_points = 0.1
 *    距离权重 w_distance = 0.1
 *    匹配度权重 w_matching = 0.8
 *
 *    遍历聚合信息序列 gather_infos, 统计计量信息
 *      计算点数 points
 *      计算距离 distance
 *      计算匹配度 matching
 *
 *    遍历聚合信息序列 gather_infos, 计量信息归一化处理
 *
 *    遍历聚合信息序列 gather_infos
 *      聚合评分: score = w_points * points_norm + w_distance * distance_norm + w_matching * matching_norm
 *
 * @param w_points in -
 * @param w_distance in -
 * @param w_matching in -
 * @param gather_infos in out - 聚合信息
 */
void ObjCloudPattern::scores(float w_matching, float w_distance, float w_points, std::vector<GatherInfo> &gather_infos)
{
  for (unsigned int i = 0; i<gather_infos.size(); i++) {
    gather_infos[i].score = gather_infos[i].points_norm * w_points +
                            gather_infos[i].distance_norm * w_distance +
                            gather_infos[i].matching_norm * w_matching;
  }
}

/** @brief 返回评分最高者
 * @param states in - 聚类块信息
 * @param gather_infos in - 聚合信息
 * @param pose_type in - 位姿输出方法
 * @param pose out - 返回聚合评分最高者位姿
 */
void ObjCloudPattern::getMaxScore(
    const std::vector<ClusterState> &states,
    const std::vector<GatherInfo> &gather_infos,
    int pose_type, Eigen::Matrix<float, 1, 9> &pose)
{
  // 获取最高得分
  // ==========
  int max_i = 0;
  float max_score = 0;
  for(unsigned int i=0; i<gather_infos.size(); i++) {
    if (gather_infos[i].score>max_score) {
      max_i = i;
      max_score = gather_infos[i].score;
    }
  }

  // 输出位姿pose
  // ===========
  if (gather_infos.size()>0) {
    //float data[3] = {L_, W_, H_};
    //int indexs[3] = {0, 1, 2};
    //sort_data(data, indexs, 3);
    if (pose_type==0) {
      // 由PCA计算
      for (int i = 0; i<9; i++) {
        pose[i] = gather_infos[max_i].bbox3d[i];
      }
    } else {
      // 由聚合点云边界确定
      float x_min = 0;
      float y_min = 0;
      float z_min = 0;
      float x_max = 0;
      float y_max = 0;
      float z_max = 0;
      getGatherCloudBoundary(states, gather_infos[max_i], x_min, y_min, z_min, x_max, y_max, z_max);
      pose[0] = (x_min + x_max) / 2;
      pose[1] = (y_min + y_max) / 2;
      pose[2] = (z_min + z_max) / 2;
      pose[3] = 0;
      pose[4] = 0;
      pose[5] = 0;
      pose[6] = x_max - x_min;
      pose[7] = y_max - y_min;
      pose[8] = z_max - z_min;
    }
  }
}

/** @brief 获取聚合点云边界
 * @param states in - 聚类块信息
 * @param info in - 聚合信息
 * @param x_min, y_min, z_min, x_max, y_max, z_max out - 聚合点云边界
 */
void ObjCloudPattern::getGatherCloudBoundary(
    const std::vector<ClusterState> &states, const GatherInfo &info,
    float &x_min, float &y_min, float &z_min,
    float &x_max, float &y_max, float &z_max)
{
  x_min = y_min = z_min =  100000000;
  x_max = y_max = z_max = -100000000;

  std::set<int>::iterator it;
  for (it=info.block_indexs.begin(); it!=info.block_indexs.end(); it++) {
    if(*it <(int)states.size()) {
      if (states[*it].x_min < x_min) {
        x_min = states[*it].x_min;
      }
      if (states[*it].x_max > x_max) {
        x_max = states[*it].x_max;
      }
      if (states[*it].y_min < y_min) {
        y_min = states[*it].y_min;
      }
      if (states[*it].y_max > y_max) {
        y_max = states[*it].y_max;
      }
      if (states[*it].z_min < z_min) {
        z_min = states[*it].z_min;
      }
      if (states[*it].z_max > z_max) {
        z_max = states[*it].z_max;
      }
    }
  }
}

/** @brief 数据排序: 降序
 * @param data in out - 数据
 * @param indexs in out - 索引
 * @param N in - 数据长度
 */
void ObjCloudPattern::sort_data(float *data, int *indexs, int N)
{
  for(int i=0; i<N-1; i++) {
    for(int j=0; j<N-1-i; j++) {
      if (data[j] < data[j+1]) {
        float temp = data[j];
        data[j] = data[j+1];
        data[j+1] = temp;
        float temp_i = indexs[j];
        indexs[j] = indexs[j+1];
        indexs[j+1] = temp_i;
      }
    }
  }
}

// 打印聚类块信息
void ObjCloudPattern::print_cluster_state(int index, const ClusterState &state)
{
  float x = (state.x_max + state.x_min)/2;
  float y = (state.y_max + state.y_min)/2;
  float z = (state.z_max + state.z_min)/2;
  float L = state.x_max - state.x_min;
  float W = state.y_max - state.y_min;
  float H = state.z_max - state.z_min;
  float distance = sqrtf(powf(x, 2.0) + powf(y, 2.0) + powf(z, 2.0));


  printf("  [%d] cluster_index=%d, x_min=%f, y_min=%f, z_min=%f, x_max=%f, y_max=%f, z_max=%f\n"
         "       x=%f, y=%f, z=%f, L=%f, W=%f, H=%f, distance=%f\n",
         index, state.cluster_index,
         state.x_min, state.y_min, state.z_min,
         state.x_max, state.y_max, state.z_max,
         x, y, z, L, W, H, distance);
}

// 打印聚合信息
void ObjCloudPattern::print_gather_info(int index, const GatherInfo &info)
{
  printf("  [%d]\n", index);
  printf("    block_indexs: ");
  std::set<int>::iterator it;
  for(it=info.block_indexs.begin(); it!=info.block_indexs.end(); it++) {printf("%d, ", *it);}
  printf("\n");
  printf("    bbox3d: ");
  for (unsigned int j=0; j<9; j++) {printf("%f, ", info.bbox3d[j]);}
  printf("\n");
  printf("    points: %d\n", info.points);
  printf("    points_norm: %f\n", info.points_norm);
  printf("    distance: %f\n", info.distance);
  printf("    distance_norm: %f\n", info.distance_norm);
  printf("    matching: %f\n", info.matching);
  printf("    matching_norm: %f\n", info.matching_norm);
  printf("    score: %f\n", info.score);
  printf("\n");
}

// 打印聚类块信息
void ObjCloudPattern::print_cluster_states(const std::vector<ClusterState> &states)
{
  printf("[ObjCloudPatter] cluster states: \n");
  for(unsigned int i=0; i<states.size(); i++) {
    print_cluster_state(i, states[i]);
  }
}
// 打印聚合信息
void ObjCloudPattern::print_gather_infos(const std::vector<GatherInfo> &infos)
{
  printf("[ObjCloudPatter] gather infos: \n");
  for(unsigned int i=0; i<infos.size(); i++) {
    print_gather_info(i, infos[i]);
  }
}

#endif // OBJ_CLOUD_PATTERN_H
