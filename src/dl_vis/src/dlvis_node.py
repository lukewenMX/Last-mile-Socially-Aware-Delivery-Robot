#!/usr/bin/env python3
import sensor_msgs.msg
import rospy
import numpy as np
import cv2
import cv_bridge
import message_filters


class dl_vis:
    def __init__(self):
        self.node_name = 'seg_trt'
        rospy.init_node(self.node_name, anonymous=True)

        self.tp_in_image = rospy.get_param('~tp_in_image', '/rgb_cam/image_raw')
        self.tp_in_seg = rospy.get_param('~tp_in_seg', '/segmentation')
        self.tp_out_vis = rospy.get_param('~tp_out_vis', '/dl_vis')

        self.img_sub = message_filters.Subscriber(self.tp_in_image,sensor_msgs.msg.Image)
        self.seg_sub = message_filters.Subscriber(self.tp_in_seg,sensor_msgs.msg.Image)
        self.vis_pub = rospy.Publisher(self.tp_out_vis,sensor_msgs.msg.Image,queue_size=5)

        self.bridge = cv_bridge.CvBridge()
        ts = message_filters.ApproximateTimeSynchronizer([self.img_sub,self.seg_sub],10,0.1,allow_headerless=True)
        ts.registerCallback(self.call_back)
    
    def call_back(self,img_msg,seg_msg):
        seg_msg.encoding = "mono8"
        ori = self.bridge.imgmsg_to_cv2(img_msg,'bgr8')
        seg = self.bridge.imgmsg_to_cv2(seg_msg)
        
        color_seg = np.zeros((seg.shape[0], seg.shape[1], 3), dtype=np.uint8)
        color_seg[seg == 1,:] = [0,255,0]
        show_img = ori * 0.5 + color_seg * 0.5
        show_img = show_img.astype(np.uint8)
        show_img_msg = self.bridge.cv2_to_imgmsg(show_img,header=seg_msg.header)
        self.vis_pub.publish(show_img_msg)

if __name__ == '__main__':
    dlvis = dl_vis()
    rospy.spin()