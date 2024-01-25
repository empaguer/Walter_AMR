from tutorial_files.move_to_table import move_to_goal
import rospy
if __name__ == '__main__':
    rospy.init_node('ommggg', anonymous=False)
    ress = move_to_goal(5,5)
    rospy.loginfo(ress)
    rospy.loginfo("this ins loginfo")