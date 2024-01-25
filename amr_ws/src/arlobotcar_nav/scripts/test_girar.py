#! /usr/bin/env python
import pandas as pd
import rospy
import time
#import buscar_orientacion
#import move_base_test
#import move_base_real
import argparse
from tutorial_files.getPosition import get_position_by_id
from tutorial_files.move_to_table import move_to_goal
from tutorial_files.new_orientation import OrientationReacher
from tutorial_files.feedback_camera import FeedbackChecker
# Example usage

if __name__ == '__main__':
    #hello()
    rospy.loginfo("shhit")

    rospy.init_node('giramiento', anonymous=False)
    orientation_class4 = OrientationReacher(2)
    orientation_class4.run()    
    #rospy.signal_shutdown("Goal reached or failed")