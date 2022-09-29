#!/usr/bin/env python3

import rospy
from semantic_segmentation import SemanticSegmentation, SemanticSegmentationException


if __name__ == '__main__':
    try:
        rospy.init_node('rospy_semantic_segmentation', log_level=rospy.INFO)
        semantic_segmentation = SemanticSegmentation()
        rospy.spin()
    except SemanticSegmentationException as e:
        rospy.logfatal('{}: Shutting down semantic_segmentation node'.format(e))
    except rospy.ROSInterruptException:
        pass
