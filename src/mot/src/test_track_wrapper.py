#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''TrackWrapper 测试
'''
import os
import numpy as np
import argparse
import cv2

# 不改变项目源码（保留原项目的包导入相对路径），把项目根目录和yolov5添加到环境变量PYTHONPATH，并切换工作目录到项目根目录
# ==> 这里的项目是指 YoloV5_DeepSORT
# ==> 否则改变工作目录会诱发import异常.
import sys
sys.path.insert(0, '/home/hjw/work/Prjs/zs_phase2_prj/src/mot/yolov5_deepsort_v6')
sys.path.insert(0, '/home/hjw/work/Prjs/zs_phase2_prj/src/mot/yolov5_deepsort_v6/yolov5')
os.chdir('/home/hjw/work/Prjs/zs_phase2_prj/src/mot/yolov5_deepsort_v6')  # 切换工作目录
from track_wrapper import TrackWrapper
# <=====工作目录切换

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--source', type=str, default='test.mp4', help='video filepath')
  args = parser.parse_args()

  path = os.getcwd()  # '/home/hjw/work/Prjs/zs_phase2_prj/src/mot/yolov5_deepsort_v6'
  print('path: ', path)
  gpus = 0
  #yolo_model = '%s/yolov5m.pt' % (path) # v5
  yolo_model = '%s/yolov5/run/CA_BiFPN.pt' % (path) # v6
  config_deepsort = '%s/deep_sort/configs/deep_sort.yaml' % (path)
  deep_sort_model = 'osnet_ibn_x1_0_MSMT17'

  track = TrackWrapper(device=0, yolo_model=yolo_model, config_deepsort=config_deepsort,
                       deep_sort_model=deep_sort_model)

  if not os.path.exists(args.source):
    print('%s not exists!' % (args.source))
    exit(0)

  cap = cv2.VideoCapture(args.source)
  cv2.namedWindow('frame', cv2.NORM_MINMAX)
  frame_idx = 0
  while(True):
      # Capture frame-by-frame
      ret, frame = cap.read()
      print('ret: ', ret)
      if not ret:
          cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
          continue

      class_ids, class_names, bboxes, scores, track_ids = track.detect(frame, frame_idx)

      if class_ids is not None:
          print('class_names: ', class_names)
          print('track_ids  : ', track_ids)
          print('bboxes     : ', bboxes)
          bboxes = np.array(bboxes).flatten().tolist()

          img_result = track.view_results(frame, class_ids, class_names, np.array(bboxes).reshape((-1, 4)),
                                          scores, track_ids)
      else:
          img_result = frame
      cv2.imshow('frame', frame)
      if cv2.waitKey(30) & 0xFF == ord('q'):
          break

  # When everything done, release the capture
  cap.release()
  cv2.destroyAllWindows()
