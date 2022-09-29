# 多目标检测/跟踪消息结构体

# 旧的数据包格式[废弃不用]
# ====================
# std_msgs/Header header
# sensor_msgs/Image  image     # 原始输入图像(可选，不发布时设置空)
# int32[]   class_ids          # 1 x objNum
# string[]  class_names        # 1 x objNum
# int32[] bboxes               # objNum x 4, Each box reprensted by [top, left, bottom, right]
# float32[] bbox3ds            # objNum x 9, Each box reprensted by [x, y, z, R, P, Y, L, W, H]
# float32[] scores             # 1 x objnum, Confidence,between 0 and 1
# int32[] track_ids            # 1 x objnum, Track IDs

# 轨迹预测
# ---------------------------
# int32 sample_len             # 采集数据长度
# int32 predict_len            # 预测数据长度
# float32[] data               # 采集+预测数据
#                              # 存储协议: [traj0, traj1,...,traj<i>, ...]
#                              #        traj<i>:    [sample_len<i>, predict_len<i>, samples<i>, predicts<i>]
#                              #        samples<i>: [sx<i>0, sy<i>0, sz<i>0, sx<i>1, sy<i>1, sz<i>1, ....]
#                              #        predicts<i>:[px<i>0, py<i>0, pz<i>0, px<i>1, py<i>1, pz<i>1, ....]
#                              #        (sx, sy, sz): 采集数据坐标(x, y, z)
#                              #        (px, py, pz): 预测数据坐标(x, y, z)

# 新的数据包格式: 没个目标独立封装一个数据包
# ===================================
std_msgs/Header header
sensor_msgs/Image  image     # 原始输入图像(可选，不发布时设置空)
int32 frame_index            # 帧续号: 统一由目标检测产生
MOT_OBJ[] objs               # 检测目标序列
