import cv2
import mediapipe as mp
from geometry_msgs.msg import Twist
import rospy

rospy.init_node('move_front', anonymous=True)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Initialize MediaPipe Hands.
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=1,
                       min_detection_confidence=0.5,
                       min_tracking_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

#función avanzar 
def move_front(velocidad):
    cmd_vel = Twist()
    cmd_vel.linear.x= velocidad
    cmd_vel_pub.publish(cmd_vel)


# Function to detect if the thumb is up and close to the camera.
def is_thumb_up_and_close(hand_landmarks):
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    thumb_ip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP]
    thumb_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP]
    index_finger_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]

    # Check if the thumb is above the index finger and its MCP joint.
    thumb_up = thumb_tip.y < thumb_ip.y < thumb_mcp.y < index_finger_mcp.y

    # Check if the hand is close to the camera based on the depth info.
    z_values = [landmark.z for landmark in hand_landmarks.landmark]
    average_z = sum(z_values) / len(z_values)
    hand_close = average_z < 0.1  # You may need to adjust this threshold based on your camera setup.

    return thumb_up and hand_close

# Function to detect if the hand is open and close to the camera.
def is_hand_open_and_close(hand_landmarks):
    # Retrieve landmark positions for fingertips and MCP joints.
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    thumb_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP]

    index_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    index_finger_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]

    middle_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    middle_finger_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP]


    ring_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
    ring_finger_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP]

    pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    pinky_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP]

    # Check if the fingers are extended.
    fingers_extended = (
        thumb_tip.y < thumb_mcp.y and
        index_finger_tip.y < index_finger_mcp.y and
        middle_finger_tip.y < middle_finger_mcp.y and
        ring_finger_tip.y < ring_finger_mcp.y and
        pinky_tip.y < pinky_mcp.y
    )

    # Check if the hand is close to the camera based on the depth info.
    z_values = [landmark.z for landmark in hand_landmarks.landmark]
    average_z = sum(z_values) / len(z_values)
    hand_close = average_z < 0.1  # You may need to adjust this threshold based on your camera setup.

    return fingers_extended and hand_close

# Initialize the camera.
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, image = cap.read()
    if not success:
        continue

    # Flip the image horizontally for a later selfie-view display, and convert the color space from BGR to RGB.
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)

    # Process the image and detect hands.
    results = hands.process(image)

    # Draw hand landmarks.
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Check if the gesture is a thumbs up and close to the camera.
            if is_thumb_up_and_close(hand_landmarks):
                cv2.putText(image, 'Thumb Up Close!', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                #PONER AQUI EL CMD A MOVERSE
                move_front(0.1)
            # Check if the hand is open and close to the camera.
            elif is_hand_open_and_close(hand_landmarks):
                cv2.putText(image, 'Open Hand Close!', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
                            cv2.LINE_AA)
                #PONER AQUI EL MOVE BASE DE DETENERSE
                move_front(0)
            else:
                cv2.putText(image, 'NONE', (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # Show the image.
    cv2.imshow('MediaPipe Hands', image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows.
cap.release()
cv2.destroyAllWindows()