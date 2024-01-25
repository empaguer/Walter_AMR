from tutorial_files.test import say_it_works
import rospy
if __name__ == '__main__':
    rospy.init_node('map_navigation', anonymous=False)
    say_it_works()
    rospy.loginfo("this ins loginfo")