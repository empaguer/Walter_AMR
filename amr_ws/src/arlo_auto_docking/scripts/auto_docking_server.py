#! /usr/bin/env python

import time
import rospy
import actionlib
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
from tf import TransformListener
from arlo_auto_docking.msg import (
    AutoDockFeedback,
    AutoDockResult,
    AutoDockAction,
)

from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Twist


import tf
from nav_msgs.msg import Odometry



from std_msgs.msg import Bool


class AutoDockingAction:
    """manages autodocking movements by using planner and move base
    """
    _feedback = AutoDockFeedback()
    _result = AutoDockResult()
    APPROACH_ANGLE = 0.1
    Z_TRANS_OFFSET = 0

    def __init__(self, name):

        # variables
        self._action_name = name

        self.theta = None
        self._r = None
        self.theta_bound = None
        self.last_aruco_detect_time = None

        self.cmd_vel_msg = Twist()

        self._tf = TransformListener()
        self.robot_position = np.array([0, 0, 0], np.dtype("float64"))

        self.switch_value = False

        # action server start
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            AutoDockAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        print("dock action server running")
        self._as.start()

        # move base action
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )

        # subscribers
        self.sub_aruco_detect = rospy.Subscriber(
            "/fiducial_transforms",
            FiducialTransformArray,
            self.aruco_detect_cb,
            queue_size=1,
        )
        self.robot_pos_subs = rospy.Subscriber(
            "/pedsim_simulator/robot_position",
            Odometry,
            self.robot_pos_callback,
        )

        self.robot_pos_subs = rospy.Subscriber(
            "/dock_switch_value",
            Bool,
            self.dock_switch_callback,
        )

        # publishers
        self.cmd_vel_pub = rospy.Publisher("/raw_cmd_vel", Twist, queue_size=10)

    # callbacks

    def dock_switch_callback(self, msg):
        """Receives value of the switch at the charging port of the robot

        Args:
            msg (std_msgs.Bool): just a boolean
        """
        self.switch_value = msg.data

    def aruco_detect_cb(self, fid_tf_array):
        """Receives aruco detection and position
        """
        try:
            # If there is no 0 index of transform, then aruco was not found
            fid_tf = fid_tf_array.transforms[0]
            # print(fid_tf)
            # rospy.loginfo("marker detected")
            [self.theta, self._r, self.theta_bounds] = self.fid2pos(
                fid_tf
            )  # for debugging
            self.last_aruco_detect_time = time.time()
        except RuntimeError:
            pass
            # rospy.loginfo("no marker detected")

    def robot_pos_callback(self, data):
        """receives and stores robot position

        Args:
            data (nav_msgs.Odometry): robot position and orientation
        """
        data_position = data.pose.pose.position
        self.robot_position = np.array(
            [data_position.x, data_position.y, data_position.z], np.dtype("float64")
        )

    def execute_cb(self, goal):
        """action server function to move robot to docking station

        Args:
            goal (): [description]
        """
        rospy.loginfo("Starting docking action")
        cancel_move_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        cancel_msg = GoalID()
        cancel_move_pub.publish(cancel_msg)
        time.sleep(1)

        if goal.request == "dock":
            while (
                self.last_aruco_detect_time is None
                or time.time() - self.last_aruco_detect_time > 1.5
            ):
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.linear.y = 0
                self.cmd_vel_msg.linear.z = 0
                self.cmd_vel_msg.angular.x = 0
                self.cmd_vel_msg.angular.y = 0
                self.cmd_vel_msg.angular.z = 0.4
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                self._feedback.feedback = "searching"
                self._as.publish_feedback(self._feedback)
            time.sleep(1)
            self._result.result = "aruco_mark_detected"
            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.linear.y = 0
            self.cmd_vel_msg.linear.z = 0
            self.cmd_vel_msg.angular.x = 0
            self.cmd_vel_msg.angular.y = 0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

            print("marker_found")
            self._feedback.feedback = "marker_found"
            self._as.publish_feedback(self._feedback)

            _t = self._tf.getLatestCommonTime("/base_footprint", "/map")
            position, quaternion = self._tf.lookupTransform("/base_footprint", "/map", _t)
            robot_offset_angle = tf.transformations.euler_from_quaternion(quaternion)[2]
            print("Robot Position:", self.robot_position)
            print("Robot Angle:", math.degrees(robot_offset_angle))
            print("Marker Distance:", self._r)
            print("Marker angle:", math.degrees(self.theta))

            self.theta = -self.theta
            print("Marker position x:", self._r * math.sin(self.theta))
            print("Marker position y:", self._r * math.cos(self.theta))

            marker_position = np.array(
                [
                    self.robot_position[0] + self._r * math.sin(self.theta),
                    self.robot_position[1] + self._r * math.cos(self.theta),
                    self.robot_position[2],
                ],
                np.dtype("float64"),
            )
            print("Marker Position", marker_position)
            time.sleep(2)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = marker_position[0] + 0.5
            goal.target_pose.pose.position.y = marker_position[1]
            goal.target_pose.pose.orientation.w = 1
            goal.target_pose.pose.orientation.z = 0
            print("sending goal")
            self.move_base_client.send_goal(goal)
            wait = self.move_base_client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                self._result.result = False
                self._as.set_succeeded(self._result)
            else:
                move_base_result = self.move_base_client.get_result()
                print(move_base_result)

            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.linear.y = 0
            self.cmd_vel_msg.linear.z = 0
            self.cmd_vel_msg.angular.x = 0
            self.cmd_vel_msg.angular.y = 0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

            while (
                self.last_aruco_detect_time is None
                or time.time() - self.last_aruco_detect_time > 1.5
            ):
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.linear.y = 0
                self.cmd_vel_msg.linear.z = 0
                self.cmd_vel_msg.angular.x = 0
                self.cmd_vel_msg.angular.y = 0
                self.cmd_vel_msg.angular.z = 0.4
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                self._feedback.feedback = "searching"
                self._as.publish_feedback(self._feedback)
            time.sleep(1)
            self._result.result = "aruco_mark_detected"
            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.linear.y = 0
            self.cmd_vel_msg.linear.z = 0
            self.cmd_vel_msg.angular.x = 0
            self.cmd_vel_msg.angular.y = 0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

            print("marker_found")
            self._feedback.feedback = "marker_found"
            self._as.publish_feedback(self._feedback)

            _t = self._tf.getLatestCommonTime("/base_footprint", "/map")
            position, quaternion = self._tf.lookupTransform("/base_footprint", "/map", _t)
            robot_offset_angle = tf.transformations.euler_from_quaternion(quaternion)[2]
            print("Robot Position:", self.robot_position)
            print("Robot Angle:", math.degrees(robot_offset_angle))
            print("Marker Distance:", self._r)
            print("Marker angle:", math.degrees(self.theta))

            self.theta = -self.theta
            print("Marker position x:", self._r * math.sin(self.theta))
            print("Marker position y:", self._r * math.cos(self.theta))

            marker_position = np.array(
                [
                    self.robot_position[0] + self._r * math.sin(self.theta),
                    self.robot_position[1] + self._r * math.cos(self.theta),
                    self.robot_position[2],
                ],
                np.dtype("float64"),
            )
            print("Marker Position", marker_position)
            time.sleep(2)

            # final goal for centering

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = marker_position[0] + 0.5
            goal.target_pose.pose.position.y = marker_position[1]
            goal.target_pose.pose.orientation.w = 0.707
            goal.target_pose.pose.orientation.z = 0.707
            print("sending goal")

            self.move_base_client.send_goal(goal)
            wait = self.move_base_client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                self._result.result = False
                self._as.set_succeeded(self._result)
            else:
                move_base_result = self.move_base_client.get_result()
                print(move_base_result)

            self.cmd_vel_msg.linear.x = -0.05
            self.cmd_vel_msg.linear.y = 0
            self.cmd_vel_msg.linear.z = 0
            self.cmd_vel_msg.angular.x = 0
            self.cmd_vel_msg.angular.y = 0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            while not self.switch_value:
                pass
            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.linear.y = 0
            self.cmd_vel_msg.linear.z = 0
            self.cmd_vel_msg.angular.x = 0
            self.cmd_vel_msg.angular.y = 0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

            self._result.result = True
            rospy.loginfo("finished action")
            self._as.set_succeeded(self._result)

    def fid2pos(self, fid_tf):
        """computes position of the robot from fiducials
        """
        q_now = [
            fid_tf.transform.rotation.x,
            fid_tf.transform.rotation.y,
            fid_tf.transform.rotation.z,
            fid_tf.transform.rotation.w,
        ]
        euler_angles = euler_from_quaternion(q_now)
        x_trans = fid_tf.transform.translation.x
        z_trans = fid_tf.transform.translation.z
        z_trans = z_trans - self.Z_TRANS_OFFSET
        theta = math.atan2(x_trans, z_trans)
        _r = math.sqrt(x_trans ** 2 + z_trans ** 2)
        if _r > 3.0:
            theta_bounds = self.APPROACH_ANGLE
        else:
            theta_bounds = _r / 30.0
        # if abs(theta)<APPROACH_ANGLE:
        # rospy.loginfo("z=%fm and x=%fm", z_trans, x_trans)
        # rospy.loginfo("Theta: %3.3f, r: %3.3f, x_trans: %3.3f, z_trans: %3.3f, x: %3.3f, y: %3.3f, z: %3.3f", theta, r, x_trans, z_trans, euler_angles[0], euler_angles[1], euler_angles[2])
        # rospy.loginfo(
        #     "Theta: %3.3f, r: %3.3f, theta_bounds: %3.3f", theta, r, theta_bounds
        # )
        return theta, _r, theta_bounds


if __name__ == "__main__":
    rospy.init_node("auto_dock_server_node")
    server = AutoDockingAction(rospy.get_name())
    rospy.spin()
