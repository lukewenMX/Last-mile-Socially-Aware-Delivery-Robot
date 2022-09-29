import rospy
import cv2
import numpy as np
from PIL import Image as Image_PIL
from yolo import YOLO
from deep_sort import preprocessing
from deep_sort import nn_matching
from deep_sort.detection import Detection
from deep_sort.detection_yolo import Detection_YOLO
from deep_sort.tracker import Tracker
from tools import generate_detections as gdet
from sidewalk_msgs.msg import BoundingBox, BoundingBoxes
import cv_bridge
from sensor_msgs.msg import Image


class PedestrianYOLOException(Exception):
    pass


class PedestrianYOLO:
    def __init__(self):
        if not rospy.has_param('pedestrian_yolo_config'):
            raise PedestrianYOLOException('No pedestrian_yolo configuration found')
        self.config = rospy.get_param('pedestrian_yolo_config')

         # initiate subscribers
        rospy.Subscriber("/camera_out/front_left/image_raw", Image, self.on_image)

        # initiate publishers
        self.pub_detection = rospy.Publisher("/detection_image", Image, queue_size=1)
        self.pub_boundingbox = rospy.Publisher("/detection_boudingbox", BoundingBoxes, queue_size=1)

        # project matrix
        self.proj_param =  np.array(rospy.get_param('proj_param', [[-0.07171913263749408, -1.011439300657605, 240.588742648954],
                                                            [ 0.04139938831247656, -1.717706203977328, 358.261876664511],
                                                            [ 0.0001293730884764893, -0.005035697541383047, 1]]))

        # Definition of the parameters
        max_cosine_distance = 0.3
        nn_budget = None
        self.nms_max_overlap = 1.0

        # Deep SORT
        model_filename = '/home/chentairan/Desktop/Sidewalk_navigation/src/pedestrian_yolo/src/model_data/mars-small128.pb'
        self.encoder = gdet.create_box_encoder(model_filename, batch_size=1)
        
        metric = nn_matching.NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
        self.tracker = Tracker(metric)

        self.tracking = True

        self.yolo = YOLO()

    def on_image(self, msg):

        """
        Callback function for the vision_cone_detector topic.
        """
        frame = cv_bridge.imgmsg_to_cv2(msg)
        image = Image_PIL.fromarray(frame[...,::-1])  # bgr to rgb
        boxes, confidence, classes = self.yolo.detect_image(image)

        if self.tracking:
            features = self.encoder(frame, boxes)

            detections = [Detection(bbox, confidence, cls, feature) for bbox, confidence, cls, feature in
                          zip(boxes, confidence, classes, features)]
        else:
            detections = [Detection_YOLO(bbox, confidence, cls) for bbox, confidence, cls in
                          zip(boxes, confidence, classes)]
        
        # Run non-maxima suppression.
        boxes = np.array([d.tlwh for d in detections])
        scores = np.array([d.confidence for d in detections])
        indices = preprocessing.non_max_suppression(boxes, self.nms_max_overlap, scores)
        detections = [detections[i] for i in indices]

        for det in detections:
            bbox = det.to_tlbr()
            score = "%.2f" % round(det.confidence * 100, 2) + "%"
            cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 0, 0), 2)
            if len(classes) > 0:
                cls = det.cls
                cv2.putText(frame, str(cls) + " " + score, (int(bbox[0]), int(bbox[3])), 0,
                            1e-3 * frame.shape[0], (0, 255, 0), 1)

        if self.tracking:
            # Call the tracker
            self.tracker.predict()
            self.tracker.update(detections)

            bboxes_msg = BoundingBoxes()

            for track in self.tracker.tracks:
                if not track.is_confirmed() or track.time_since_update > 1:
                    continue
                bbox_msg = BoundingBox()
                bbox = track.to_tlbr()
                cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 255, 255), 2)
                cv2.putText(frame, "ID: " + str(track.track_id), (int(bbox[0]), int(bbox[1])), 0,
                            1e-3 * frame.shape[0], (0, 255, 0), 1)

                bbox_msg.xmin, bbox_msg.ymin, bbox_msg.xmax, bbox_msg.ymax = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
                bbox_msg.id = track.track_id
                bbox_msg.Class = "pedestrian"
                bboxes_msg.bounding_boxes.append(bbox_msg)
            bboxes_msg.header = msg.header
            self.pub_boundingbox.publish(bboxes_msg)
        m = cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        m.header = msg.header
        self.pub_detection.publish(m)
