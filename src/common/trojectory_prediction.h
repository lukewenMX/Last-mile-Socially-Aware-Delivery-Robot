#ifndef TROJECTORY_PREDICTION_H
#define TROJECTORY_PREDICTION_H

/** @轨迹预测
 *
 * 使用范例
 * =======
 * // 定义轨迹预测对象
 * TrajectoryPrediction traj;
 * // 设置参数
 * traj.setSampleMaxLen(100);   // 最大采集100个样本
 * traj.setPredictMaxLen(100);  // 最大预测100个数据
 * traj.setMaxMisCount(1);      // 最大缺失次数: 大于阈值将从轨迹队列中删除
 *
 * // 轨迹预测
 * double timestamp = 123456.789; // 秒
 * std::vector<int> class_ids;    // 检测结果: 类别ID序列
 * std::vector<std::string> class_names; // 检测结果: 类别名称序列
 * std::vector<int> bboxes;               // 检测结果: 2D bbox
 * std::vector<float> bbox3ds;            // 检测结果: 3D bbox
 * std::vector<float> scores;             // 检测结果: 置信度
 * std::vector<int> track_ids;            // 检测结果: 跟踪ID
 * traj.predict(timestamp, class_ids, class_names, bboxes, bbox3ds, scores, track_ids);
 *
 * // 打印轨迹数据
 * traj.print_infos(traj.getData());
 *
 * // 保存轨迹数据
 * char *out_path = "/tmp/trajs";
 * int cur_frame_index = 100; // 当前帧序号
 * traj.save_trajs(traj.getData(), out_path, cur_frame_index);
 *
 * 图形绘制
 * =======
 * 看 <zs_phase2_prj>/demos/demo-draw_traj.py
 *
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <Eigen/Eigen>

class TrajectoryPrediction
{
public: // 类型定义
  typedef struct _TrackInfo
  {
    int         class_id;           // 类别ID
    std::string class_name;         // 类别名称
    int         bbox[4];            // 2D检测框: (x_min, y_min, x_max, y_max)
    float       score;              // 得分
    int         track_id;           // 跟踪ID
    std::queue<std::vector<double>/*10*/> samples;   // 采集数据队列: [(x,y,z,R,P,Y,L,W,H,timestamp), ...]
    std::queue<std::vector<double>/*10*/> predicts;  // 预测数据队列: [(x,y,z,R,P,Y,L,W,H,timpstamp), ...]

    int  mis_count;                  // 缺失统计
    bool hit;                        // 命中标记
  }TrackInfo;

public:
  TrajectoryPrediction();
  ~TrajectoryPrediction();
public:
  // 设置/获取数据采集最大长度
  void setSampleMaxLen(unsigned int len);
  unsigned int  getSampleMaxLen();
  unsigned int  getSampleCurLen(int track_id);

  // 设置/获取数据预测最大长度
  void setPredictMaxLen(unsigned int len);
  unsigned int  getPredictMaxLen();
  unsigned int  getPredictCurLen(int track_id);

  // 设置/获取最大缺失次数
  void setMaxMisCount(int n);
  int  getmaxMisCount();

//  // 方差计算
//  void caculate_std(const std::vector<float> &data, float &mean, float &std_val);
//  // 噪声过滤
//  void samples_filter(int data_len=5);

  // 轨迹预测
  void predict(double timestamp /*second*/,
               const std::vector<int> &class_ids,
               const std::vector<std::string> &class_names,
               const std::vector<int> &bboxes,
               const std::vector<float> &bbox3ds,
               const std::vector<float> &scores,
               const std::vector<int> &track_ids);
  // 获取数据: 原始数据
  const std::map<int, TrackInfo> &getData();
  // 初始化 info
  static void init_info(TrajectoryPrediction::TrackInfo &info);
  // 打印 info
  static void print_info(const TrajectoryPrediction::TrackInfo &info, bool print_traj=true);
  // 打印 infos
  static void print_infos(const std::map<int, TrajectoryPrediction::TrackInfo> &infos);
  // 保存轨迹
  static void save_traj(const TrajectoryPrediction::TrackInfo &info, const char *out_path, int frame_index);
  static void save_trajs(const std::map<int, TrajectoryPrediction::TrackInfo> &infos, const char *out_path, int frame_index);

private:

private:
  unsigned int sample_max_len_ ; // 数据采集最大长度
  unsigned int predict_max_len_; // 数据预测最大长度
  int max_mis_count_;            // 最大缺失次数
  std::map<int/*track_id*/, TrackInfo> data_; // 数据缓存
};


TrajectoryPrediction::TrajectoryPrediction()
{

}
TrajectoryPrediction::~TrajectoryPrediction()
{

}

/** @brief 设置最大采集长度
 */
void TrajectoryPrediction::setSampleMaxLen(unsigned int len)
{
  sample_max_len_ = len;
}

/** @brief 获取最大采集长度
 */
unsigned int TrajectoryPrediction::getSampleMaxLen()
{
  return sample_max_len_;
}

/** @brief 获取当前采集长度
 */
unsigned int TrajectoryPrediction::getSampleCurLen(int track_id)
{
  if (data_.find(track_id)==data_.end()) {
    return 0;
  } else {
    return data_[track_id].samples.size();
  }
}

/** @brief 设置最大预测长度
 */
void TrajectoryPrediction::setPredictMaxLen(unsigned int len)
{
  predict_max_len_ = len;
}

/** @brief 获取最大预测长度
 */
unsigned int TrajectoryPrediction::getPredictMaxLen()
{
  return predict_max_len_;
}

/** @brief 获取当前预测长度
 */
unsigned int TrajectoryPrediction::getPredictCurLen(int track_id)
{
  if (data_.find(track_id)==data_.end()) {
    return 0;
  } else {
    return data_[track_id].predicts.size();
  }
}

/** @brief 设置最大缺失次数
 */
void TrajectoryPrediction::setMaxMisCount(int n)
{
  max_mis_count_ = n;
}

/** @brief 获取最大缺失次数
 */
int TrajectoryPrediction::getmaxMisCount()
{
  return max_mis_count_;
}

// 获取数据: 原始数据
const std::map<int, TrajectoryPrediction::TrackInfo> &TrajectoryPrediction::getData()
{
  return data_;
}



// 初始化 info
void TrajectoryPrediction::init_info(TrajectoryPrediction::TrackInfo &info)
{
  for(int i=0; i<4; i++) {info.bbox[i] = 0;}
  info.class_id = -1;
  info.class_name = "";
  info.hit = 0;
  info.mis_count = 0;
  info.score = 0;
  info.track_id = -1;
}


// 打印 info
void TrajectoryPrediction::print_info(const TrajectoryPrediction::TrackInfo &info, bool print_traj)
{
  printf("  print info\n");
  printf("  ===========\n");
  printf("  class_id: %d\n", info.class_id);
  printf("  class_name: %s\n", info.class_name.c_str());
  printf("  bbox: [%d, %d, %d, %d]\n", info.bbox[0], info.bbox[1], info.bbox[2], info.bbox[3]);
  printf("  hit: %d\n", info.hit);
  printf("  mis_count: %d\n", info.mis_count);
  printf("  score: %f\n", info.score);
  printf("  track_id: %d\n", info.track_id);

  if (print_traj) {
    printf("  samples: \n");
    std::queue<std::vector<double>> samples = info.samples;
    int index = 0;
    while(samples.size()>0) {
      std::vector<double> traj_node = samples.front();
      if (traj_node.size() == 10) {
        printf("    [%d] %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", index,
               traj_node[0], traj_node[1], traj_node[2],
               traj_node[3], traj_node[4], traj_node[5],
               traj_node[6], traj_node[7], traj_node[8],
               traj_node[9]);
      } else {
        printf("    [%d] traj_node.size!=10\n",  index);
      }
      index ++;
      samples.pop();
    }

    printf("  predicts: \n");
    std::queue<std::vector<double>> predicts = info.predicts;
    index = 0;
    while(predicts.size()>0) {
      std::vector<double> traj_node = predicts.front();
      if (traj_node.size() == 10) {
        printf("    [%d] %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", index,
               traj_node[0], traj_node[1], traj_node[2],
               traj_node[3], traj_node[4], traj_node[5],
               traj_node[6], traj_node[7], traj_node[8],
               traj_node[9]);
      } else {
        printf("    [%d] traj_node.size!=10\n",  index);
      }
      index ++;
      predicts.pop();
    }
  } else {
    printf("  samples: data_len=%ld \n", info.samples.size());
    printf("  predicts: data_len=%ld \n", info.predicts.size());
  }
  printf("\n");
}

// 打印 infos
void TrajectoryPrediction::print_infos(const std::map<int, TrajectoryPrediction::TrackInfo> &infos)
{
  printf("print infos\n");
  printf("===========\n");
  std::map<int, TrajectoryPrediction::TrackInfo>::const_iterator it;
  for (it=infos.begin(); it!=infos.end(); it++) {
    print_info(it->second);
  }
}

//// 方差计算
//void TrajectoryPrediction::caculate_std(const std::vector<float> &data, float &mean_val, float &std_val)
//{
//  mean_val = 0;
//  std_val = 0;
//  // 数据数据检查
//  if (data.size() <=0) {
//    return;
//  }

//  // 计算均值
//  for (unsigned int i=0; i<data.size(); i++) {
//    mean_val += data[i];
//  }
//  mean_val /= data.size();

//  // 计算方差
//  for(unsigned  int i=0; i<data.size(); i++) {
//    std_val += powf(data[i]-mean_val, 2.0);
//  }
//  std_val = sqrtf(std_val);
//}

//// 噪声过滤
//void TrajectoryPrediction::samples_filter(int data_len)
//{
//  std::vector<float> vec_x(5); // 最后添加的无个x坐标点

//  std::map<int, TrackInfo>::iterator it;
//  for (it = data_.begin(); it!=data_.end(); it++) {
//    // 最小数据长度检测
//    if ((int)it->second.samples.size() == data_len) {
//      // 小数据处理
//      for(int i=0; i<data_len; i++) {
//        vec_x[i] = it->second.samples.front()[0];
//        it->second.samples.pop();
//      }
//      float mean_val = 0;
//      float std_val = 0;
//      caculate_std(vec_x, mean_val, std_val);

//      // 3倍方差修正
//      for (int i=0; i<data_len; i++) {
//        if (fabs(vec_x[i] - mean_val) > 3 * std_val) {
//          if (i == 0 && i < data_len-1) {
//            vec_x[0] = vec_x[1];
//          } else if(i == data_len-1) {
//            vec_x[i] = vec_x[i-1];
//          } else if (i>0 && i < data_len-1) {
//            vec_x[i] = (vec_x[i-1] + vec_x[i+1])/2;
//          }
//        }
//      }

//      // 数据回填

//    } else if((int)it->second.samples.size() > data_len) {
//      // 多数据处理
//    }

//    // FIR: 3倍方差规则过滤奇异噪声点, 只对x轴处理
//  }
//}

/** @brief 轨迹预测
 *  @param timestamp in - 时间戳(秒), msg.header.stamp.toSec()
 *  @param class_ids in - 类别ID序列
 *  @param class_names in - 类别名称序列
 *  @param bboxes in - 2D检测框序列
 *  @param bbox3ds in - 位姿估算序列
 *  @param scores in - 检测可信度序列
 *  @param track_ids in - 跟踪ID序列
 */
void TrajectoryPrediction::predict(double timestamp,
                                   const std::vector<int> &class_ids,
                                   const std::vector<std::string> &class_names,
                                   const std::vector<int> &bboxes,
                                   const std::vector<float> &bbox3ds,
                                   const std::vector<float> &scores,
                                   const std::vector<int> &track_ids)
{
  // 合法性检测
  if (class_ids.size() != class_names.size() ||
      class_ids.size() != bboxes.size()/4 ||
      class_ids.size() != bbox3ds.size()/9 ||
      class_ids.size() != scores.size() ||
      class_ids.size() != track_ids.size())
  {
    printf("[E:%s:%d][TrajectoryPrediction::predict] vector size not matches:\n"
           "class_ids.size: %ld\n"
           "class_names.size: %ld\n"
           "bboxes.size: %ld\n"
           "bbox3ds.size: %ld\n"
           "scores.size: %ld\n"
           "track_ids.size: %ld\n"
           "timestamp: %f\n", __FILE__, __LINE__,
           class_ids.size(), class_names.size(), bboxes.size(), bbox3ds.size(), scores.size(), track_ids.size(),
           timestamp);
    return;
  }
  // 初始化hit
  std::map<int, TrackInfo>::iterator it;
  for (it = data_.begin(); it!=data_.end(); it++) {
    it->second.hit = false;
  }

  // 遍历检测数据
  for(unsigned int i=0; i<class_ids.size(); i++) {
    // 提取 3DBBox
    std::vector<double> traj_node; // (x, y, z, R, P, Y, L, W, H, timestamp)
    traj_node.resize(10);
    for(unsigned int j=0; j<9; j++) {
      traj_node[j] = bbox3ds[i * 9 + j];
    }
    traj_node[9] = timestamp; // 标注时间戳
    if (traj_node[0]==0 && traj_node[1]==0 && traj_node[2]==0) {
      continue;
    }

    // 首次Track初始化
    if (data_.find(track_ids[i]) == data_.end()) {
      TrackInfo info;
      info.class_id = class_ids[i];
      info.class_name = class_names[i];
      info.score = scores[i];
      info.track_id = track_ids[i];
      info.bbox[0] = bboxes[i*4+0];
      info.bbox[1] = bboxes[i*4+1];
      info.bbox[2] = bboxes[i*4+2];
      info.bbox[3] = bboxes[i*4+3];
      info.hit = true;
      info.mis_count = 0;
      data_.insert(std::pair<int, TrackInfo>(track_ids[i], info));
    }

    // 设置 hit 状态
    data_[track_ids[i]].hit = true;

    // 入队
    data_[track_ids[i]].samples.push(traj_node);

    if (data_[track_ids[i]].samples.size()>sample_max_len_) {
      // 大于最大长度： 出队
      data_[track_ids[i]].samples.pop();
    }    
  }

  // 缺失处理: hit, miscount
  for (it = data_.begin(); it!=data_.end();) {
    if (!it->second.hit) {
      it->second.mis_count++;
    }

    if (it->second.mis_count>max_mis_count_) {
      // 清除
      std::map<int, TrackInfo>::iterator it_tmp = it;
      it_tmp++;
      data_.erase(it);
      it = it_tmp;
    } else {
      // 下一个
      it++;
    }
  }

  // 轨迹预测
  // ......
}

// 保存轨迹
void TrajectoryPrediction::save_traj(const TrajectoryPrediction::TrackInfo &info,
                                      const char *out_path, int frame_index)
{
  // samples
  // -------
  if (info.samples.size()>0) {
    printf("  save samples:\n");
    char sz_file[512] = {0};
    char str_node[1024] = {0};
    sprintf(sz_file, "%s/samples_%06d_%06d.txt", out_path, frame_index, info.track_id);
    std::ofstream fout ;
    fout.open(sz_file) ;
    if (fout.is_open()) {
      std::queue<std::vector<double>> samples = info.samples;
      int index = 0;
      while(samples.size()>0) {
        std::vector<double> traj_node = samples.front();
        if (traj_node.size() == 10) {
          sprintf(str_node, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                 traj_node[0], traj_node[1], traj_node[2],
                 traj_node[3], traj_node[4], traj_node[5],
                 traj_node[6], traj_node[7], traj_node[8],
                 traj_node[9]);
          fout << str_node;

        } else {
          printf("    [%d] traj_node.size!=10\n",  index);
        }
        index ++;
        samples.pop();
      }
      fout.close() ;
      printf("[I:%s:%d][TrajectoryPrediction::save_traj][samples] save to '%s'\n", __FILE__, __LINE__, sz_file);
    } else {
      printf("[E:%s:%d][TrajectoryPrediction::save_traj][samples] can't open '%s'\n", __FILE__, __LINE__, sz_file);
    }
  }

  // predicts
  // -------
  if (info.predicts.size()>0) {
    printf("  save predicts:\n");
    char sz_file[512] = {0};
    char str_node[1024] = {0};
    sprintf(sz_file, "%s/predicts_%06d_%06d.txt", out_path, frame_index, info.track_id);
    std::ofstream fout ;
    fout.open(sz_file) ;
    if (fout.is_open()) {
      std::queue<std::vector<double>> predicts = info.predicts;
      int index = 0;
      while(predicts.size()>0) {
        std::vector<double> traj_node = predicts.front();
        if (traj_node.size() == 10) {
          sprintf(str_node, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                 traj_node[0], traj_node[1], traj_node[2],
                 traj_node[3], traj_node[4], traj_node[5],
                 traj_node[6], traj_node[7], traj_node[8],
                 traj_node[9]);
          fout << str_node;

        } else {
          printf("    [%d] traj_node.size!=10\n",  index);
        }
        index ++;
        predicts.pop();
      }
      fout.close() ;
      printf("[I:%s:%d][TrajectoryPrediction::save_traj][predicts] save to '%s'\n", __FILE__, __LINE__, sz_file);
    } else {
      printf("[E:%s:%d][TrajectoryPrediction::save_traj][predicts] can't open '%s'\n", __FILE__, __LINE__, sz_file);
    }
  }
}
void TrajectoryPrediction::save_trajs(const std::map<int, TrajectoryPrediction::TrackInfo> &infos,
                                      const char *out_path, int frame_index)
{
  printf("save infos\n");
  printf("===========\n");
  std::map<int, TrajectoryPrediction::TrackInfo>::const_iterator it;
  for (it=infos.begin(); it!=infos.end(); it++) {
    save_traj(it->second, out_path, frame_index);
  }
}

#endif // TROJECTORY_PREDICTION_H
