#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

save_dir = "/home/rosuser/catkin_ws/src/video_recorder/photos"
os.makedirs(save_dir, exist_ok=True)

filename = input("Enter filename (without extension): ") + ".jpg"

def camera_callback(msg):
   bridge = CvBridge()
   cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
   image_path = os.path.join(save_dir, filename)
   cv2.imwrite(image_path, cv_image)
   rospy.signal_shutdown('Image saved')

rospy.init_node('camera_capture')
rospy.Subscriber('locobot/camera/color/image_raw', Image, camera_callback)
rospy.spin()