#!/usr/bin/env python3
'''
多目标跟踪

Publications:
    /objs_detect/mot [mmtrack/MOT]

Subscriptions:
    /sensors_calib/image_rect [sensor_msgs/Image]

Services:
            
Actions:

envirement:
    YOLOV5_DEEPSORT_PATH - YoloV5_DeepSORT 项目根目录

Parametors:
    ~node_name
        节点名称，启动多个目标检测程序时的唯一性标识, default: mot
    ~verbose
        日志输出等级
    ~gpus
        GPU配置， default="0"
    ~memory
        显存配置比例, default="0.1"
    ~frame_id
        ROS Message header.frame_id
    ~conf_thres
        置信度阈值, default='.5'
    ~iou_thres
        IOU阈值, default='0.5
    ~tp_in_image
        订阅的图像主题, default: /sensors_calib/image_rect
    ~tp_out_mot
        发布的多目标检测与跟踪主题, default: /objs_detect/mot
    ~tp_out_vis
        发布多目标检测结果: 调试用， verbose>2 时有效
    ~yolo_model
        yolov5配置模型文件路径, default: $(find mot)/yolov5_deepsort/yolov5m.pt
    ~config_deepsort
        DeepSORT配置文件, default: $(find mot)/yolov5_deepsort/deep_sort/configs/deep_sort.yaml
    ~deep_sort_model
        DeepSORT模型文件配置路径, default: $(find mot)/yolov5_deepsort/deep_sort/deep/checkpoint/osnet_ibn_x1_0_MSMT17

Verbose[debug]:
    verbose=2
    /dbg/img_result[snesor_msgs/Image] - 由dbg_pub_img_result发布的目标检测结果(图像)

Arguments:

'''
import os
import rospy
import message_filters
import sensor_msgs.msg
import rosgraph_msgs.msg
import std_msgs.msg
import numpy as np
import ros_numpy
import cv2
import mot.msg

# 不改变项目源码（保留原项目的包导入相对路径），把项目根目录和yolov5添加到环境变量PYTHONPATH，并切换工作目录到项目根目录
# ==> 这里的项目是指 YoloV5_DeepSORT
# ==> 否则改变工作目录会诱发import异常.
import sys
print('YOLOV5_DEEPSORT_PATH: ', os.getenv('YOLOV5_DEEPSORT_PATH')) # '/home/hjw/work/Prjs/zs_phase2_prj/src/mot/yolov5_deepsort'
sys.path.insert(0, os.getenv('YOLOV5_DEEPSORT_PATH'))
sys.path.insert(0, '%s/yolov5' % (os.getenv('YOLOV5_DEEPSORT_PATH')))
os.chdir(os.getenv('YOLOV5_DEEPSORT_PATH'))  # 切换工作目录
from track_wrapper import TrackWrapper
# <=====工作目录切换

# 目标检测
class Tracker(object):
    def __init__(self):
        self.node_name = 'mmtrack'
        rospy.init_node(self.node_name, anonymous=True)

        # 获取参数
        # -------
        self.verbose = rospy.get_param('~verbose', 0)
        gpus = rospy.get_param('~gpus', '0')
        memory = rospy.get_param('~memory', 0.2)
        tp_in_image = rospy.get_param('~tp_in_image', '/rgb_cam/image_raw/compressed')
        tp_out_mot = rospy.get_param('~tp_out_mot', '/obj_detect/mot')
        tp_out_vis = rospy.get_param('~tp_out_vis', '/vis/img_result')
        yolo_model = rospy.get_param('~yolo_model', 'yolov5m.pt')
        config_deepsort = rospy.get_param('~config_deepsort', 'yolov5_deepsort/deep_sort/configs/deep_sort.yaml')
        deep_sort_model = rospy.get_param('~deep_sort_model', 'deep_sort/deep/checkpoint/osnet_ibn_x1_0_MSMT17')
        frame_id = rospy.get_param('~frame_id', 'livox_frame')
        conf_thres = rospy.get_param('~conf_thres', 0.5)
        iou_thres = rospy.get_param('~iou_thres', 0.5)

        # 打印参数
        print('')
        print('Parameters')
        print('==========')
        print('             node: ', self.node_name)
        print('          verbose: ', self.verbose)
        print('             gpus: ', gpus)
        print('           memory: ', memory)
        print('      tp_in_image: ', tp_in_image)
        print('       tp_out_mot: ', tp_out_mot)
        print('       tp_out_vis: ', tp_out_vis)
        print('       yolo_model: ', yolo_model)
        print('  config_deepsort: ', config_deepsort)
        print('  deep_sort_model: ', deep_sort_model)
        print('         frame_id: ', frame_id)
        print('        conf_thrs: ', conf_thres)
        print('         iou_thrs: ', iou_thres)
        print('')

        self.tp_in_image = tp_in_image  # debug

        self.frame_idx = 0
        self.frame_id = frame_id
        self.track = TrackWrapper(device=gpus,
                                  yolo_model=yolo_model,
                                  config_deepsort=config_deepsort,
                                  deep_sort_model=deep_sort_model,
                                  conf_thres=conf_thres, iou_thres=iou_thres)

        self.pub = rospy.Publisher(tp_out_mot, mot.msg.MOT, queue_size=10)
        self.sub = rospy.Subscriber(tp_in_image, sensor_msgs.msg.Image, self.callback, queue_size=2)
        print('Subscriber topic: ', tp_in_image)
        print('Publisher  topic: ', tp_out_mot)
        # 发布目标检测结果(图像)
        self.dbg_pub_img_result = rospy.Publisher(tp_out_vis, sensor_msgs.msg.Image)

        # 阻塞，直到 Ctrl+C
        # rospy.spin()

    def callback(self, img_ros, *args):
        '''
        @param img_ros  - sensor_msgs.msg.CompressedImage
        @param *args    - None
        '''
        # 图像解码
        img = ros_numpy.numpify(img_ros)
        # ## HSV 变换测试 begin
        # if self.tp_in_image.endswith('rgb'):
        #     img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  #cv2.cvtColor(p1,p2) 是颜色空间转换函数，p1是需要转换的图片，p2是转换成何种格式，此处转换为HSV格式
        #     img_hsv[:, :, -1] *= 0.2
        #     img = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)
        # ## HSV 变换测试 end

        # 推理
        class_ids, class_names, bboxes, scores, track_ids = self.track.detect(img, self.frame_idx)
        # 发布
        msg = mot.msg.MOT()
        msg.header = img_ros.header
        if self.frame_id != '':
            msg.header.frame_id = self.frame_id
        msg.image = img_ros
        msg.frame_index = self.frame_idx
        if class_ids is not None:
            if self.verbose > 0:
                print('--------------------------')
                print('  class_ids: ', class_ids)
                print('class_names: ', class_names)
                print('     scores: ', scores)
                print('  track_ids: ', track_ids)
                print('     bboxes: ', bboxes)

            # 数据包MOT数据填充
            for i in range(len(class_ids)):
                obj = mot.msg.MOT_OBJ()
                obj.class_id = class_ids[i]
                obj.class_name = class_names[i]
                obj.bbox = bboxes[i]  # yolov5输出是 [x, y, w, h] 还是 [x1, y1, x2, y2]？ 存疑，从数据表现看为后者，待深入分析代码，此处暂定为后者方式输出
                obj.bbox3d = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # init
                if (i <len(scores)):
                    obj.score = scores[i]
                else:
                    # 有时候数目不对齐，原因待查！！！
                    obj.score = 0.5
                    print('[WARNING] MOG_OBJ fill data, miss score of index ', i)
                obj.track_id = track_ids[i]
                msg.objs.append(obj)

        if self.verbose > 2 and self.dbg_pub_img_result is not None:
            # verbose=2
            if class_ids is not None:
                img_result = self.track.view_results(img, class_ids, class_names, np.array(bboxes).reshape((-1, 4)),
                                                     scores, track_ids)
            else:
                img_result = img
            ros_img = ros_numpy.msgify(msg_type=sensor_msgs.msg.Image, numpy_obj=img_result, encoding='rgb8')
            self.dbg_pub_img_result.publish(ros_img)
        self.pub.publish(msg)
        self.frame_idx += 1


    def __del__(self):
        pass

if __name__ == '__main__':
    cls = Tracker()
    rospy.spin()
