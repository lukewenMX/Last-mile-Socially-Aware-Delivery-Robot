import time
import cv2
import numpy as np
from mmdeploy_python import Detector
from trackers.ocsort_tracker.ocsort import OCSort

class mot_tracker:
    def __init__(self, path_to_model, gpu_index = 0, conf_thresh = 0.3, track_thresh = 0.6, iou_thresh = 0.3):
        self.path = path_to_model
        self.track_thresh = track_thresh
        self.iou_thresh = iou_thresh
        self.conf_thresh = conf_thresh
        self.use_byte = False
        print('[MOT]: track_thresh:',self.track_thresh)
        print('[MOT]: iou_thresh:',self.iou_thresh)
        print('[MOT]: conf_thresh:',self.conf_thresh)
        self.detector = Detector(model_path=self.path,device_name='cuda',device_id=gpu_index)
        self.tracker = OCSort(det_thresh=self.track_thresh,iou_threshold=self.iou_thresh,use_byte=self.use_byte)
    
    def draw_detect(self,draw_img,bbox,label,label_color):
        cv2.rectangle(draw_img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color= label_color, thickness=2)
        labelSize = cv2.getTextSize(label + '0', cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
        if bbox[1] - labelSize[1] - 3 < 0:
            cv2.rectangle(draw_img,(bbox[0], bbox[1] + 2),(bbox[0] + labelSize[0], bbox[1] + labelSize[1] + 3),color=label_color,thickness=-1)
            cv2.putText(draw_img, label,(bbox[0], bbox[1] + labelSize[1] + 3),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 0, 0),thickness=1)
        else:
            cv2.rectangle(draw_img,(bbox[0], bbox[1] - labelSize[1] - 3),(bbox[0] + labelSize[0], bbox[1] - 3),color=label_color,thickness=-1)
            cv2.putText(draw_img, label,(bbox[0], bbox[1] - 3),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 0, 0),thickness=1)

    def draw_results(self,draw_img,results):
        for target in results:
            id = target[4]
            self.draw_detect(draw_img,target[0:4].astype(int),'person-' + str(int(id)),(255,0,255))

    def update(self,img):
        bboxes, labels, masks = self.detector(img)
        detect_objs = []
        for index in range(len(bboxes)):
            if labels[index] != 0:
                continue
            bbox = bboxes[index]

            score = bbox[4]
            if score < self.conf_thresh:
                continue
            detect_objs.append(bbox[0:5])

        if len(detect_objs) == 0:
            detect_array = np.zeros((0,5))
        else:
            detect_array = np.array(detect_objs)
        mot_results = self.tracker.update(detect_array)
        return mot_results


if __name__ == '__main__':
    now_mot = mot_tracker('/home/cx/dissertation/Sidewalk_navigation/src/mot_oc/src/bin/yoloxs_trt')
    video = cv2.VideoCapture('/home/cx/mot_tests/test_1.mp4')
    tot = 0
    cnt = 0
    while (video.isOpened()):
        ret, img = video.read()
        st = time.time()
        mot_results = now_mot.update(img)
        ed = time.time()

        tot += ed - st
        cnt += 1
        if cnt == 100:
            avg_time = tot * 1000 / cnt
            print('avg_infer_time:',avg_time,' ms.')
            cnt = 0
            tot = 0
        
        now_mot.draw_results(img,mot_results)
        cv2.imshow('mot_img',img)
        cv2.waitKey(10)