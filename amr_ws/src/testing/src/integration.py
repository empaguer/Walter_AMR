import rospy
import math
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def poseCallback(pose_message):
    global x
    global y,z,yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

def rotate(velocity_publisher,angular_speed_degree, relative_angle_degree,clockwise):
    velocity_message = Twist()
    angular_speed = math.radians(abs(angular_speed_degree))
    if(clockwise):
        velocity_message.angular.z = - abs(angular_speed)
    else:
        velocity_message.angular.z =   abs(angular_speed)
    loop_rate = rospy.Rate(10)
    t0 = rospy.Time.now().to_sec()
    while True:
        rospy.loginfo(" turtlesim rotating!")
        velocity_publisher.publish(velocity_message)
        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()
        if(current_angle_degree > relative_angle_degree):
            rospy.loginfo("reached")
            break
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)
def move(velocity_publisher,speed,distance, is_forward):
    velocity_message = Twist()
    global x, y
    x0 = x
    y0 = y
    if(is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)
    distance_moved = 0.0
    loop_rate = rospy.Rate(10)
    while True:
        rospy.loginfo("Turtlesim moving forward")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        distance_moved = abs(   math.sqrt(((x-x0)**2)+((y-y0)**2)))
        print(distance_moved)
        if not (distance_moved<distance):
            rospy.loginfo("reached")
            break
    velocity_message.linear.x =0
    velocity_publisher.publish(velocity_message)
def go_to_goal(velocity_publisher, x_goal, y_goal):
    global x
    global y, yaw
    velocity_message = Twist()
    loop_rate = rospy.Rate(10)
    while(True):
        K_linear = 0.5
        distance = abs(math.sqrt((x_goal-x)**2)+ ((y_goal-y)**2))
        linear_speed = distance*K_linear
        if(linear_speed > 1.0):
            print("way to fast")
        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        print("x=",x," y=", y, " distance goal = ", distance)
        if(distance<0.1):
            break
    velocity_message.linear.x =0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)
def setDesiredOrientation(publisher,speed_in_degree, desired_angle_degree):
    relative_angle_radians = math.radians(desired_angle_degree) - yaw
    clockwise = 0
    if relative_angle_radians < 0:
        clockwise =1
    else:
        clockwise =0
    print("relative angle : ", math.degrees(relative_angle_radians))
    print("desired_angle:", desired_angle_degree)
    rotate(publisher,speed_in_degree,math.degrees(abs(relative_angle_radians)),clockwise)
def spiral(velocity_publisher, rk, wk):
    vel_msg = Twist()
    loop_rate = rospy.Rate(1)
    while((x<10.5) and (y< 10.5)):
        rk = rk + 0.5
        vel_msg.linear.x = rk
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x =0
        vel_msg.angular.y =0
        vel_msg.angular.z = wk
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
    vel_msg.linear.x = 0
    vel_msg.angular.z =0
    velocity_publisher.publish(vel_msg)
if __name__ == "__main__":
    try:
        rospy.init_node('turtlesim_motion_pose',anonymous=True)
        cmd_vel_topic = "/turtle1/cmd_vel"
        position_topic = "/turtle1/pose"
        velocity_publisher = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)
        pose_subscriber = rospy.Subscriber(position_topic,Pose,poseCallback)
        time.sleep(2)
        #move(velocity_publisher,1.0,3.0,False)
        #rotate(velocity_publisher,10,90,True)
        #go_to_goal(velocity_publisher,10,11)
        #setDesiredOrientation(velocity_publisher,10,80)
        spiral(velocity_publisher,0,2)
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")
        pass