import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
x_velocity = 0
turn_velocity = 0
extra_x_vel = 0
extra_turn_vel = 0
def joy_callback(data):
    global x_velocity, turn_velocity, extra_x_vel, extra_turn_vel
    global cmd_vel_pub
    twist = Twist()
    raw_x_velocity = 0.3 * data.axes[1]
    raw_turn_velocity = 0.5 * data.axes[3]
    x_velocity = round(raw_x_velocity,2)
    turn_velocity = round(raw_turn_velocity,2)
    rospy.loginfo(f"x_velocity {x_velocity} and turn_velocity {turn_velocity}")    
    twist.linear.x = x_velocity
    twist.angular.z = turn_velocity
    cmd_vel_pub.publish(twist)
    
if __name__ == '__main__':
    rospy.init_node('improved_joy_to_cmdvel')
    cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    rospy.Subscriber('joy',Joy,joy_callback)
    rospy.spin()