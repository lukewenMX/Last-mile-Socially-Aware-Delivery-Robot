#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''多目标检测与跟踪(在track.py基础上修改)
    为了避免不改动Yolov5_DeepSORT源码时诱发的import异常，使用时需要以下处理:
    1. 添加YoloV5_DeepSORT和Yolov5到环境变量PYTHONPATH
    2. 切换工作目录为 YoloV5_DeepSORT根目录

    详细看下面的测试代码

# TrackWrapper 测试
# =================
import os
import numpy as np
import argparse
import cv2

# 不改变项目源码（保留原项目的包导入相对路径），把项目根目录和yolov5添加到环境变量PYTHONPATH，并切换工作目录到项目根目录
# ==> 这里的项目是指 YoloV5_DeepSORT
# ==> 否则改变工作目录会诱发import异常.
import sys
sys.path.insert(0, '/home/hjw/work/Prjs/zs_phase2_prj/src/mot/yolov5_deepsort')
sys.path.insert(0, '/home/hjw/work/Prjs/zs_phase2_prj/src/mot/yolov5_deepsort/yolov5')
os.chdir('/home/hjw/work/Prjs/zs_phase2_prj/src/mot/yolov5_deepsort')  # 切换工作目录
from track_wrapper import TrackWrapper
# <=====工作目录切换

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--source', type=str, default='test.mp4', help='video filepath')
  args = parser.parse_args()

  path = os.getcwd()  # '/home/hjw/work/Prjs/zs_phase2_prj/src/mot/yolov5_deepsort'
  gpus = 0
  yolo_model = '%s/yolov5m.pt' % (path)
  config_deepsort = '%s/deep_sort/configs/deep_sort.yaml' % (path)
  # deep_sort_model 约定
  #      1。 deep_sort_model 为模型名称，不含目录，不含扩展名, e.g. osnet_ibn_x1_0_MSMT17
  #      2。 存放约定: <mot>/yolov5_deepsort/deep_sort/deep/checkpoint/<deep_sort_model>.pth
  deep_sort_model = 'osnet_ibn_x1_0_MSMT17'

  track = TrackWrapper(device=0, yolo_model=yolo_model, config_deepsort=config_deepsort,
                       deep_sort_model=deep_sort_model)

  if not os.path.exists(args.source):
    print('%s not exists!' % (args.source))
    exit(0)

  cap = cv2.VideoCapture(args.source)

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
'''
# limit the number of cpus used by high performance libraries
import os

import numpy as np

os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import sys
sys.path.insert(0, './yolov5')

import argparse
import os
import platform
import shutil
import time
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn

from yolov5.models.experimental import attempt_load
from yolov5.utils.downloads import attempt_download
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.datasets import LoadImages, LoadStreams
from yolov5.utils.general import (LOGGER, check_img_size, non_max_suppression, scale_coords, 
                                  check_imshow, xyxy2xywh, increment_path)
from yolov5.utils.torch_utils import select_device, time_sync
from yolov5.utils.plots import Annotator, colors
from deep_sort.utils.parser import get_config
from deep_sort.deep_sort import DeepSort

from yolov5.utils.augmentations import letterbox

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # yolov5 deepsort root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

class TrackWrapper:
    def __init__(self, device='0',
                 yolo_model='yolov5m.pt',
                 config_deepsort='deep_sort/configs/deep_sort.yaml',
                 deep_sort_model='osnet_ibn_x1_0_MSMT17',
                 classes=None,
                 agnostic_nms=False,
                 augment=False,
                 half=True,
                 dnn=True,
                 conf_thres = 0.5,
                 iou_thres = 0.5,
                 imgsz=[640,640],
                 max_det=1000):
        '''
        @param device - GPU('0','1',...) 或 ‘cpu'
        @param config_deepsort - DeepSORT配置文件
        @param deep_sort_model - DeepSORT模型名称, e.g. osnet_ibn_x1_0_MSMT17
        @param augment - augmented inference
        @param conf_thres - object confidence threshold
        @param iou_thres - IOU threshold for NMS
        @param half - 是否使用半精度模式
        @param yolo_model - YOLO模型
        @param dnn - use OpenCV DNN for ONNX inference
        @param imgsz - inference size h,w
        @param max_det - maximum detection per image
        @param classes - filter by class: --class 0, or --class 16 17
        @param agnostic_nms - class-agnostic NMS
        '''
        device = select_device(device)
        # initialize deepsort
        cfg = get_config()
        cfg.merge_from_file(config_deepsort)
        deepsort = DeepSort(deep_sort_model,
                            device,
                            max_dist=cfg.DEEPSORT.MAX_DIST,
                            max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                            max_age=cfg.DEEPSORT.MAX_AGE, n_init=cfg.DEEPSORT.N_INIT, nn_budget=cfg.DEEPSORT.NN_BUDGET,
                            )

        # Initialize
        half &= device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        device = select_device(device)
        model = DetectMultiBackend(yolo_model, device=device, dnn=dnn)
        stride, names, pt, jit, _ = model.stride, model.names, model.pt, model.jit, model.onnx
        imgsz = check_img_size(imgsz, s=stride)  # check image size

        # Half
        half &= pt and device.type != 'cpu'  # half precision only supported by PyTorch on CUDA
        if pt:
            model.model.half() if half else model.model.float()

        # Get names and colors
        names = model.module.names if hasattr(model, 'module') else model.names
        if pt and device.type != 'cpu':
            model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.model.parameters())))  # warmup

        cudnn.benchmark = True  # set True to speed up constant image size inference

        self.names = names
        self.model = model
        self.deepsort = deepsort
        self.imgsz = imgsz
        self.stride = stride
        self.device = device
        self.max_det = max_det

        self.pt = pt
        self.jit = jit
        self.half = half
        self.augment = augment
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.classes = classes
        self.agnostic_nms = agnostic_nms


    def detect(self, image, frame_id):
        '''多目标检测与跟踪
        @param image - 输入图像
        @param frame_id - 帧序号
        @return (class_ids, class_names, bboxes, scores, track_ids)
            class_ids - 类别ID, 1xobjNum
            class_names - 类别名称, 1xobjNum
            bboxes - 检测边框, objNum x 4
            scores - 置信度, 1xobjNum
            track_ids - 跟踪ID, 1xobjNum
        '''
        pass

        # 图像预处理
        img = [letterbox(image, self.imgsz, stride=self.stride, auto=True)[0]]
        # Stack
        img = np.stack(img, 0)
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        pred = self.model(img, augment=self.augment, visualize=False)

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

        det = pred[0] if len(pred)>0 else None

        class_ids = None
        class_names = None
        bboxes = None
        scores = None
        track_ids = None

        if det is not None and len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(
                img.shape[2:], det[:, :4], image.shape).round()

            xywhs = xyxy2xywh(det[:, 0:4])
            confs = det[:, 4]
            clss = det[:, 5]

            # pass detections to deepsort
            outputs = self.deepsort.update(xywhs.cpu(), confs.cpu(), clss.cpu(), image)

            if len(outputs) > 0:
                class_ids = outputs[:, 5].astype(np.int32)
                class_names = [self.names[c] for c in class_ids]
                bboxes = [box for box in outputs[:, 0:4]]
                scores = confs
                track_ids = [track_id for track_id in outputs[:, 4]]
        else:
            self.deepsort.increment_ages()
            LOGGER.info('No detections')

        return class_ids, class_names, bboxes, scores, track_ids

    def view_results(self, image, class_ids, class_names, bboxes, scores, track_ids):
        '''结果显示
        @param image - 输入图像
        @param frame_id - 帧序号
        @param class_ids - 类别ID, 1xobjNum
        @param class_names - 类别名称, 1xobjNum
        @param bboxs - 检测边框, objNum x 4
        @param scores - 置信度, 1xobjNum
        @param track_ids - 跟踪ID, 1xobjNum
        @return 绘制结果
        '''
        annotator = Annotator(image, line_width=2, pil=not ascii)
        num = len(class_ids) if len(class_ids) <= len(scores) else len(scores)
        for i in range(num):
            bbox = bboxes[i]
            id = track_ids[i]
            cls = class_ids[i]
            # print('scores: ', scores, ', i: ', i)
            conf = scores[i]
            label = f'{id} {class_names[i]} {conf:.2f}'
            annotator.box_label(bbox, label, color=colors(cls, True))

        # Stream results
        im0 = annotator.result()
        return im0
        # cv2.imshow('img', im0)
        # if cv2.waitKey(1) == ord('q'):  # q to quit
        #     raise StopIteration

def detect(opt):
    # 创建多目标检测/跟踪对象，初始化内部参数
    # --------------------------------
    track = TrackWrapper()

    # Dataloader
    dataset = LoadStreams(opt.source, img_size=track.imgsz, stride=track.stride, auto=track.pt and not track.jit)
    bs = len(dataset)  # batch_size

    for frame_idx, (path, img, im0s, vid_cap, s) in enumerate(dataset):
        # 处理一帧数据
        class_ids, class_names, bboxes, scores, track_ids = track.detect(im0s[0], frame_idx)
        # 显示检测/跟踪结果
        if not class_ids is None:
            track.view_results(im0s[0], class_ids, class_names, bboxes, scores, track_ids)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--yolo_model', nargs='+', type=str, default='yolov5m.pt', help='model.pt path(s)')
    parser.add_argument('--deep_sort_model', type=str, default='osnet_ibn_x1_0_MSMT17')
    parser.add_argument('--source', type=str, default='0', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--output', type=str, default='inference/output', help='output folder')  # output folder
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.5, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--fourcc', type=str, default='mp4v', help='output video codec (verify ffmpeg support)')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--show-vid', action='store_true', help='display tracking video results')
    parser.add_argument('--save-vid', action='store_true', help='save video tracking results')
    parser.add_argument('--save-txt', action='store_true', help='save MOT compliant results to *.txt')
    # class 0 is person, 1 is bycicle, 2 is car... 79 is oven
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 16 17')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--evaluate', action='store_true', help='augmented inference')
    parser.add_argument("--config_deepsort", type=str, default="deep_sort/configs/deep_sort.yaml")
    parser.add_argument("--half", action="store_true", help="use FP16 half-precision inference")
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detection per image')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    parser.add_argument('--project', default=ROOT / 'runs/track', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand

    with torch.no_grad():
        detect(opt)
