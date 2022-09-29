#!/usr/bin/env python3

import rospy
from prediction.prediction import Prediction, PredictionException


if __name__ == '__main__':
    try:
        rospy.init_node('rospy_prediction', log_level=rospy.INFO)
        prediction = Prediction()
        rospy.spin()
    except PredictionException as e:
        rospy.logfatal('{}: Shutting down prediction node'.format(e))
    except rospy.ROSInterruptException:
        pass
