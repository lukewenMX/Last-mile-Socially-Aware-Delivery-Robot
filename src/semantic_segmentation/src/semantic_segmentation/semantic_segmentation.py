import rospy
import numpy as np
# from semantic_segmentation import cv_bridge
import cv2
import cv_bridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import time


class SemanticSegmentationException(Exception):
    pass


class SemanticSegmentation:
    def __init__(self):
        if not rospy.has_param('semantic_segmentation_config'):
            raise SemanticSegmentationException('No semantic_segmentation configuration found')

        MODEL = rospy.get_param('~model', 'mnv2_bdd100k_driveable_513')
        TOPIC_IMAGE = rospy.get_param('~topic_image', '/fl_cam/color/image_raw')

        TOPIC_SEMANTIC = rospy.get_param('~topic_semantic', 'semantic')
        TOPIC_SEMANTIC_COLOR = rospy.get_param('~topic_semantic_color', 'semantic_color')
        TOPIC_SEMANTIC_FUSION_COLOR = rospy.get_param('~topic_semantic_fusion_color', 'semantic_fusion_color')

        self.proj_param =  np.array(rospy.get_param('proj_param', [[-0.02257165579461952, -0.8409525473827573, 214.1766970191965],
                                                                    [ 0.04497342193716747, -1.746228841643724, 436.1616454988515],
                                                                    [ 0.0001186079744282781, -0.004304741552503774, 1]]))
        rospy.Subscriber(TOPIC_IMAGE, Image, self.on_image, queue_size=1)

        self.pub_semantic = rospy.Publisher(TOPIC_SEMANTIC, Image, queue_size=1)
        self.pub_semantic_color = rospy.Publisher(TOPIC_SEMANTIC_COLOR, Image, queue_size=1)
        self.pub_semantic_fusion_color = rospy.Publisher(TOPIC_SEMANTIC_FUSION_COLOR, Image, queue_size=1)
        self.pub_occupancymap = rospy.Publisher("/occupancy_grid", OccupancyGrid, queue_size=1)
        self.pub_local_goal = rospy.Publisher("/local_goal", PoseStamped, queue_size=1)

        self.model = getattr(__import__('models', globals(), locals(), fromlist=[MODEL]),
                             MODEL).Model()
        rospy.set_param("semantic_categories", self.model.categories)

        self.downsampling_rate = 1
        self.bridge = cv_bridge.CvBridge()
        self.cnt = 0

    def on_image(self, msg):
        self.cnt = self.cnt + 1
        if self.cnt % self.downsampling_rate == 0:
            self.cnt = 0
        else:
            return

        header = msg.header

        time_start=time.time()
        image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        semantic = self.model.infer([image])[0]

        time_seg=time.time()

        #self.extract_boundary(semantic)

        time_end=time.time()
        # print(f"seg time cost: {time_seg - time_start} s")
        # print(f'boundary time cost: {time_end - time_seg} s')

        if self.pub_semantic.get_num_connections() > 0:
            m = self.bridge.cv2_to_imgmsg(semantic.astype(np.uint8), encoding='mono8')
            m.header.stamp.secs = header.stamp.secs
            m.header.stamp.nsecs = header.stamp.nsecs
            self.pub_semantic.publish(m)

        if self.pub_semantic_color.get_num_connections() > 0:
            m = self.bridge.cv2_to_imgmsg(self.model.color_map[semantic.astype(np.uint8)], encoding='bgr8')
            m.header.stamp.secs = header.stamp.secs
            m.header.stamp.nsecs = header.stamp.nsecs
            self.pub_semantic_color.publish(m)

        if self.pub_semantic_fusion_color.get_num_connections() > 0:
            m = self.bridge.cv2_to_imgmsg(
                cv2.addWeighted(self.model.color_map[semantic.astype(np.uint8)], 0.5, self.bridge.imgmsg_to_cv2(msg,'bgr8'), 1,
                                0), encoding='bgr8')
            m.header.stamp.secs = header.stamp.secs
            m.header.stamp.nsecs = header.stamp.nsecs
            self.pub_semantic_fusion_color.publish(m)

    def extract_boundary(self, semantic):
        # IPM
        gridscacle = 10
        gridsize = 0.05
        L = (int)(2 * gridscacle / gridsize)

        semantic = semantic.astype("uint8")
        dst = cv2.warpPerspective(semantic, self.proj_param, dsize=(L, L))

        # Convert coordinate system
        dst = np.flip(np.flip(dst.T, axis=1), axis=0)

        # Close operator
        kernel = np.ones((10, 10), dtype=np.uint8)
        dst = cv2.morphologyEx(dst, cv2.MORPH_CLOSE, kernel)

        # Extract Contours & calculate mass center
        ret, thresh = cv2.threshold(dst, 0, 255, 0, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        image = np.zeros((400, 400, 3), np.uint8)

        if len(contours) > 0:
            contours_length = np.array([len(contour) for contour in contours])
            index = np.argmax(contours_length)
            cnts = cv2.drawContours(image, contours[index], -1, (0, 255, 0), 1)
        else:
            return
        kpCnt = len(contours[0])

        x = 0
        y = 0

        for kp in contours[0]:
            x = x+kp[0][0]
            y = y+kp[0][1]

        center_x = x/kpCnt
        center_y = y/kpCnt
        pos_x = center_y * gridsize
        pos_y = gridscacle - center_x * gridsize

        # cv2.circle(image, (np.uint8(np.ceil(center_x)), np.uint8(np.ceil(center_y))), 1, (0, 0, 255), 3)
        # cv2.namedWindow("Result", cv2.WINDOW_NORMAL)
        # cv2.imshow("Result", image)
        # cv2.waitKey(1)

        # Publish local Goal
        local_goal = PoseStamped()
        local_goal.header.frame_id = "base_link"
        local_goal.header.stamp = rospy.Time.now()
        local_goal.pose.position.x = pos_x
        local_goal.pose.position.y = pos_y
        self.pub_local_goal.publish(local_goal)

        # Generate occupancy grid 
        cost_map = OccupancyGrid()
        cost_map.header.frame_id = "base_link"
        cost_map.header.stamp = rospy.Time.now()
        cost_map.info.resolution = 0.05
        cost_map.info.width = L
        cost_map.info.height = L
        cost_map.info.origin.position.y = -gridscacle
        cost_map.info.origin.position.x = 0

        cost_map.data = np.where(dst.flatten() == 1, 0, 100)

        self.pub_occupancymap.publish(cost_map)
