#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import threading
import time
from datetime import datetime

# Setup save directory
save_dir = "/home/rosuser/catkin_ws/src/video_recorder/photos"
os.makedirs(save_dir, exist_ok=True)

# Initialize variables
bridge = CvBridge()
latest_image = None
image_count = 0

def camera_callback(msg):
    """Store the latest image from the camera"""
    global latest_image
    latest_image = msg

def save_image():
    """Save the latest image with timestamp"""
    global latest_image, image_count
    
    if latest_image is None:
        print("No image received yet. Waiting...")
        return False
    
    try:
        # Convert ROS image to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(latest_image, "bgr8")
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"image_{timestamp}_{image_count}.jpg"
        image_count += 1
        
        # Save the image
        image_path = os.path.join(save_dir, filename)
        cv2.imwrite(image_path, cv_image)
        print(f"Image saved as {filename}")
        return True
    except Exception as e:
        print(f"Error saving image: {str(e)}")
        return False

def input_thread_function():
    """Handle user input in a separate thread"""
    global running
    
    print("Press 'q' and Enter to capture an image, or type 'exit' and press Enter to quit.")
    while running:
        user_input = input("")  # Empty prompt, waiting for any key
        if user_input.lower() == 'exit':
            print("Exiting...")
            running = False
            rospy.signal_shutdown("User requested exit")
            break
        elif user_input.lower() == 'q':  # Check for 'q' input
            print("Taking photo...")
            save_image()
        # Ignore other inputs

# Initialize ROS node
rospy.init_node('camera_capture')
rospy.Subscriber('locobot/camera/color/image_raw', Image, camera_callback)

# Set up and start input thread
running = True
input_thread = threading.Thread(target=input_thread_function)
input_thread.daemon = True
input_thread.start()

# Main loop
try:
    while not rospy.is_shutdown() and running:
        rospy.sleep(0.1)  # Sleep to avoid consuming too much CPU
except KeyboardInterrupt:
    running = False
    print("Interrupted by user, shutting down...")

print("Exiting camera capture program")