#!/usr/bin/env python3
import mot.msg
import sensor_msgs.msg
import rospy
import numpy as np
import cv2
import cv_bridge
from mot_trt import mot_tracker

class Tracker:
    def __init__(self):
        self.node_name = 'mot_ocsort'
        rospy.init_node(self.node_name, anonymous=True)
        
        self.verbose = rospy.get_param('~verbose', 0)
        self.gpu_index = rospy.get_param('~gpu_index', 0)
        
        self.tp_in_image = rospy.get_param('~tp_in_image', '/rgb_cam/image_raw/compressed')
        self.tp_out_mot = rospy.get_param('~tp_out_mot', '/obj_detect/mot')
        self.tp_out_vis = rospy.get_param('~tp_out_vis', '/vis/img_result')

        self.yolox_model = rospy.get_param('~yolox_model', 'bin/yoloxs_trt')

        self.frame_id = rospy.get_param('~frame_id', 'cam_frame')
        self.conf_thres = rospy.get_param('~conf_thres', 0.3)
        self.track_thres = rospy.get_param('~track_thres', 0.6)
        self.iou_thres = rospy.get_param('~iou_thres', 0.3)

        self.pub_mot = rospy.Publisher(self.tp_out_mot, mot.msg.MOT, queue_size=10)
        self.sub_img = rospy.Subscriber(self.tp_in_image, sensor_msgs.msg.Image, self.callback, queue_size=1)
        print('[MOT] Subscriber topic: ', self.tp_in_image)
        print('[MOT] Publisher  topic: ', self.tp_out_mot)
        self.frame_index = 0

        # 发布目标检测结果(图像)
        self.pub_img_result = rospy.Publisher(self.tp_out_vis, sensor_msgs.msg.Image, queue_size = 10)
        self.bridge = cv_bridge.CvBridge()
        self.tracker = mot_tracker(self.yolox_model,self.gpu_index,self.conf_thres,self.track_thres,self.iou_thres)


    def callback(self,img_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg,'bgr8')
        mot_results = self.tracker.update(img)
        
        mot_msg = mot.msg.MOT()
        mot_msg.header = img_msg.header
        mot_msg.header.frame_id = self.frame_id
        mot_msg.frame_index = self.frame_index
        # mot_msg.image = img_msg
        for target in mot_results:
            obj = mot.msg.MOT_OBJ()
            obj.class_id = 0
            obj.class_name = 'person'
            obj.bbox = target[0:4].astype(int)
            obj.bbox3d = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # init
            obj.score = 0.9
            obj.track_id = int(target[4])
            mot_msg.objs.append(obj)
        self.pub_mot.publish(mot_msg)
        self.frame_index += 1
        #发布可视化图像
        if self.verbose > 0:
            self.tracker.draw_results(img,mot_results)
            vis_img_msg = self.bridge.cv2_to_imgmsg(img,header=img_msg.header)
            self.pub_img_result.publish(vis_img_msg)
        #使用opencv显示
        if self.verbose > 1:
            cv2.imshow('mot',img)
            cv2.waitKey(1)
        

if __name__ == '__main__':
    trk = Tracker()
    rospy.spin()