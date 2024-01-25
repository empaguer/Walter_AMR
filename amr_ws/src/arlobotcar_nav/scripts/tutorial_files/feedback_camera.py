#!/usr/bin/env python
import cv2
import mediapipe as mp
from geometry_msgs.msg import Twist
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
class FeedbackChecker:
    def __init__(self):
        rospy.init_node("omg",anonymous=True)
        self.is_feedback_shown = False
        self.mp_hands = mp.solutions.hands
        self.bridge = CvBridge()
        self.hands = self.mp_hands.Hands(static_image_mode=False,
                       max_num_hands=1,
                       min_detection_confidence=0.5,
                       min_tracking_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils
        self.thumbs_up_start_time = None
        self.elapsed_time = 0
        
        #rospy.init_node('feedback_node', anonymous=True)

        #rospy.init_node('position_reach_timer', anonymous=True)
        #self.timer = rospy.Timer(rospy.Duration(30), self.timer_callback, oneshot=True)
    def confirm_thumbs_up(self):
        print("Thumbs Up Confirmed!")

    def camera_callback(self,image_msg):
        #print("being called")
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                # Check if the gesture is a thumbs up and close to the camera.
                if self.is_thumb_up_and_close(hand_landmarks):
                    cv2.putText(image, 'Thumb Up Close!', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                    if self.thumbs_up_start_time is None:
                        self.thumbs_up_start_time = time.time()
                        print("Detecting Thumbs Up...")

                    self.elapsed_time = time.time() - self.thumbs_up_start_time
                    if self.elapsed_time >= 5.0:
                        self.confirm_thumbs_up()
                        self.thumbs_up_start_time = None
                        self.is_feedback_shown = True
                else:
                    self.thumbs_up_start_time = None
        else:
            self.thumbs_up_start_time = None

        # Show the image.
        print(self.elapsed_time)
        #print(flag)
        #print("nothing here")
        cv2.imshow('MediaPipe Hands', image)
        cv2.waitKey(1)
    def run(self):

        # Code to command the robot to move to the target position
        rospy.loginfo("run")
        rospy.Subscriber('d435/color/image_raw', Image, self.camera_callback)
        rate = rospy.Rate(10)  # Control loop rate (10 Hz)
        while not rospy.is_shutdown() and not self.is_feedback_shown:
            rospy.loginfo("here")
            rate.sleep()
        # Stop the timer if it's still running

        print("feedback shown") 

        # Additional code for cleanup or further actions
    def is_thumb_up_and_close(self,hand_landmarks):
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP]
        thumb_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP]
        index_finger_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]

        # Check if the thumb is above the index finger and its MCP joint.
        thumb_up = thumb_tip.y < thumb_ip.y < thumb_mcp.y < index_finger_mcp.y

        # Check if the hand is close to the camera based on the depth info.
        z_values = [landmark.z for landmark in hand_landmarks.landmark]
        average_z = sum(z_values) / len(z_values)
        hand_close = average_z < 0.1  # You may need to adjust this threshold based on your camera setup.

        return thumb_up and hand_close
    


if __name__ == "__main__":
    position_reach_timer = FeedbackChecker()
    position_reach_timer.run()
