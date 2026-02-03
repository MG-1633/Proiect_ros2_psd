#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image # <--- Added Image
from cv_bridge import CvBridge # <--- To read Robot's Camera
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import mediapipe as mp
import time
import numpy as np

class HideAndSeekNode(Node):
    def __init__(self):
        super().__init__('hide_and_seek_node')
        
        # 1. PUBS & SUBS
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        
        # Subscribe to Robot's Camera (Only works on Waffle Pi)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, qos)
        self.bridge = CvBridge()
        
        self.timer = self.create_timer(0.1, self.timer_loop)
        
        # 2. AI & Webcam Setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0) # Your Laptop Camera
        
        # 3. Variables
        self.state = "IDLE" 
        self.count_start_time = 0
        self.laser_ranges = [] 
        self.target_found = False # Did we see the Red Object?

        self.get_logger().info("Hide & Seek: Vision & Lidar Active!")

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges

    def camera_callback(self, msg):
        """
        Runs whenever the Robot sends a camera image.
        We look for RED objects here.
        """
        if self.state != "SEEKING":
            return # Save CPU if we aren't looking

        try:
            # Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert to HSV color space (easier to detect colors)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define RED color range
            # Red wraps around 0/180 in HSV, so we need two masks
            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            
            mask = mask1 + mask2
            
            # Find Contours (blobs of red)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500: # If the blob is big enough (close enough)
                    self.target_found = True
                    self.get_logger().info("TARGET FOUND!")
                    
        except Exception as e:
            pass # Image might be corrupt or bridge failed

    def count_fingers(self, landmarks, label):
        """
        Counts fingers, adjusting logic based on which hand it is.
        label: "Left" or "Right" (from MediaPipe)
        """
        finger_tips = [8, 12, 16, 20]  # Index, Middle, Ring, Pinky
        count = 0
        
        
        
        thumb_tip_x = landmarks[4].x
        thumb_knuckle_x = landmarks[3].x
        
        if label == "Left":
            # For a "Left" hand, the thumb is to the RIGHT of the knuckle
            if thumb_tip_x > thumb_knuckle_x:
                count += 1
        else: # Label is "Right"
            # For a "Right" hand, the thumb is to the LEFT of the knuckle
            if thumb_tip_x < thumb_knuckle_x:
                count += 1
            
        
        for tip in finger_tips:
            # If tip is higher (Y is smaller) than the PIP joint (tip - 2)
            if landmarks[tip].y < landmarks[tip - 2].y:
                count += 1
                
        return count

    def get_scan_regions(self):
        if not self.laser_ranges: return 10.0, 10.0, 10.0
        clean_ranges = [r if (r > 0.05 and r < 10.0) else 10.0 for r in self.laser_ranges]
        front_left = clean_ranges[0:30]
        front_right = clean_ranges[-30:]
        return min(front_left + front_right), min(clean_ranges[30:90]), min(clean_ranges[270:330])

    def timer_loop(self):
        ret, frame = self.cap.read()
        if not ret: return
        
        frame = cv2.flip(frame, 1)
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)
        
        fingers = 0
        hand_label = "Right" # Default assumption
        
        # --- A. DETECT HANDS ---
        if results.multi_hand_landmarks:
            for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # 1. Get the Label FIRST
                if results.multi_handedness:
                    # MediaPipe gives us "Left" or "Right"
                    hand_label = results.multi_handedness[idx].classification[0].label
                
                # 2. Pass the label to count_fingers
                fingers = self.count_fingers(hand_landmarks.landmark, hand_label)

        

        # --- B. UPDATE STATE ---
        
        # 1 Finger -> Start Counting
        if fingers == 1 and self.state != "COUNTING":
            self.state = "COUNTING"
            self.count_start_time = time.time()
            self.target_found = False
            
        # 5 Fingers -> Go Seek (Autonomous)
        elif fingers == 5:
            self.state = "SEEKING"
            
        # Fist -> Stop
        elif fingers == 0:
            self.state = "IDLE"
            
        # --- NEW LOGIC: 2 FINGERS (Directional) ---
        elif fingers == 2:
            # Note: Because we flipped the image (mirror), the labels might be swapped.
            # Usually: 'Left' label in code = Physical Right Hand (due to flip)
            # You might need to swap these words if it feels backwards!
            if hand_label == "Left": 
                self.state = "HINT_LEFT" # Physical Right Hand
            else:
                self.state = "HINT_RIGHT"  # Physical Left Hand

        # If we found the target, override
        if self.target_found and self.state == "SEEKING":
            self.state = "FOUND"

        # --- C. ROBOT BEHAVIOR ---
        msg = Twist()
        front_d, left_d, right_d = self.get_scan_regions()
        
        if self.state == "IDLE":
            pass
            
        elif self.state == "COUNTING":
            elapsed = time.time() - self.count_start_time
            if elapsed < 5.0:
                msg.angular.z = 0.5
                cv2.putText(frame, f"COUNTING: {int(5-elapsed)}", (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 4)
            else:
                self.state = "IDLE"

        elif self.state == "SEEKING":
            if front_d > 0.6:
                msg.linear.x = 0.2
            else:
                msg.linear.x = 0.0
                if left_d > right_d: msg.angular.z = 0.5
                else: msg.angular.z = -0.5
                
        # --- DIRECTIONAL COMMANDS ---
        elif self.state == "HINT_LEFT":
            msg.angular.z = 0.5 # Turn Left
            
        elif self.state == "HINT_RIGHT":
            msg.angular.z = -0.5 # Turn Right
            
        elif self.state == "FOUND":
            msg.linear.x = 0.0
            msg.angular.z = 1.0
            cv2.putText(frame, "FOUND YOU!", (100, 300), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4)

        self.publisher_.publish(msg)

        # --- D. VISUALIZATION ---
        # Show the Hand Label on screen so you can debug "Left" vs "Right"
        cv2.putText(frame, f"Hand: {hand_label}", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        cv2.putText(frame, f"State: {self.state}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Fingers: {fingers}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        cv2.imshow("Game Master", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HideAndSeekNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()