#!/usr/bin/env python3
import sensor_msgs.msg
import rospy
import numpy as np
import cv2
import cv_bridge
from mmdeploy_python import Segmentor


class seg_trt:
    def __init__(self):
        self.node_name = 'seg_trt'
        rospy.init_node(self.node_name, anonymous=True)

        self.classes = ('road', 'sidewalk', 'building', 'wall', 'fence', 'pole',
               'traffic light', 'traffic sign', 'vegetation', 'terrain', 'sky',
               'person', 'rider', 'car', 'truck', 'bus', 'train', 'motorcycle',
               'bicycle')

        self.palette = [[128, 64, 128], [244, 35, 232], [70, 70, 70], [102, 102, 156],
               [190, 153, 153], [153, 153, 153], [250, 170, 30], [220, 220, 0],
               [107, 142, 35], [152, 251, 152], [70, 130, 180], [220, 20, 60],
               [255, 0, 0], [0, 0, 142], [0, 0, 70], [0, 60, 100],
               [0, 80, 100], [0, 0, 230], [119, 11, 32]]

        self.verbose = rospy.get_param('~verbose', 0)
        self.tp_in_image = rospy.get_param('~tp_in_image', '/rgb_cam/image_raw')
        self.tp_out_seg = rospy.get_param('~tp_out_seg', '/semantic')
        self.tp_out_vis = rospy.get_param('~tp_out_vis', '/seg_img')
        self.gpu_index = rospy.get_param('~gpu_index',0)
        self.model_path = rospy.get_param('~model', 'bin/sdtc_trt')

        self.pub_seg = rospy.Publisher(self.tp_out_seg, sensor_msgs.msg.Image, queue_size = 5)
        self.pub_seg_vis = rospy.Publisher(self.tp_out_vis, sensor_msgs.msg.Image, queue_size = 5)
        self.sub_img = rospy.Subscriber(self.tp_in_image, sensor_msgs.msg.Image, self.callback, queue_size=1)
        print('[SEG] Model: ',self.model_path)
        print('[SEG] Subscriber topic: ', self.tp_in_image)
        print('[SEG] Publisher  topic: ', self.tp_out_seg)
        self.bridge = cv_bridge.CvBridge()
        self.segmentor = Segmentor(model_path=self.model_path, device_name ='cuda', device_id = 0)

    def callback(self, img_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg,'bgr8')
        seg = self.segmentor(img).astype(np.uint8)
        # seg[seg == 0] = 1
        
        seg_msg = self.bridge.cv2_to_imgmsg(seg,header=img_msg.header)
        self.pub_seg.publish(seg_msg)

        # print(seg.dtype)
        if self.verbose > 0:
            color_seg = np.zeros((seg.shape[0], seg.shape[1], 3), dtype=np.uint8)
            for label in range(len(self.classes)):
                color_seg[seg == label, :] = self.palette[label]
            
            color_seg = color_seg[..., ::-1]
            show_img = img * 0.5 + color_seg * 0.5
            show_img = show_img.astype(np.uint8)
            show_img_msg = self.bridge.cv2_to_imgmsg(show_img,header=img_msg.header)
            self.pub_seg_vis.publish(show_img_msg)
        if self.verbose > 1:
            cv2.imshow('seg',show_img)
            cv2.waitKey(1)

if __name__ == '__main__':
    seg = seg_trt()
    rospy.spin()