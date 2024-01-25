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
    if(data.axes[7]==1):
        x_velocity = 0.25 
        turn_velocity = 0
    elif(data.axes[7]==-1):
        x_velocity = -0.25 
        turn_velocity =0
    elif(data.axes[6]==1):
        x_velocity =0
        turn_velocity = 0.25 
    elif(data.axes[6]==-1):
        x_velocity =0
        turn_velocity = -0.25 
    elif(data.buttons[0]==1):
        x_velocity=0
        turn_velocity=0
    elif(data.buttons[4]==1):
       extra_x_vel += 0.05
       extra_turn_vel += 0.05
    elif(data.buttons[5]==1):
       extra_x_vel -= 0.05
       extra_turn_vel -= 0.05
    print(round(data.axes[1],3))    
    twist.linear.x = x_velocity
    twist.angular.z = turn_velocity
    cmd_vel_pub.publish(twist)
    
if __name__ == '__main__':
    rospy.init_node('joy_to_cmdvel')
    cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    rospy.Subscriber('joy',Joy,joy_callback)
    rospy.spin()