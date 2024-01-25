import rospy
from std_msgs.msg import String
def callback(data):
    #print('callback here')
    rospy.loginfo(rospy.get_caller_id() + "I Heard %s", data.data)
    #print('Omg')
def listener():
    rospy.init_node('listener', anonymous = True)
    #print('done')
    rospy.Subscriber('chatter',String, callback)
    #print('subs')
    rospy.spin()
    #print(' finished spin')
if __name__ == '__main__':
    print('hello')
    listener()