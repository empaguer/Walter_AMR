#! /usr/bin/env python
import pandas as pd
import rospy
import time
#import buscar_orientacion
#import move_base_test
#import move_base_real

from tutorial_files.feedback_camera import FeedbackChecker
from tutorial_files.new_orientation import OrientationReacher

# Example usage

if __name__ == '__main__':
    #hello()
    #rospy.loginfo("helllo")
    #rospy.init_node('justaa', anonymous=False)
    feedback_class = FeedbackChecker()
    feedback_class.run()
    if(feedback_class.is_feedback_shown):
        orientation_class = OrientationReacher(2)
        orientation_class.run()
        print(orientation_class.is_position_reached)
    #rospy.signal_shutdown("Goal reached or failed")