# 多目标检测/跟踪消息 目标结构体

std_msgs/Header header

# 基础信息
int32     class_id         # scalar
string    class_name       # scalar
int32[]   bbox             # [top, left, bottom, right]
float32[] bbox3d           # [x, y, z, R, P, Y, L, W, H]
float32   score            # scalar
int32     track_id         # scalar

# 轨迹预测
float64[] samples        # 采集数据序列: [(x, y, z, R, P, Y, L, W, H, timestamp), ...]
float64[] predicts       # 预测数据序列: [(x, y, z, R, P, Y, L, W, H, timestamp), ...]