#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import datetime

class VideoRecorder:
    def __init__(self):
        rospy.init_node('video_recorder', anonymous=True)

        # Video settings
        self.video_path = rospy.get_param('~video_path')
        self.fps = rospy.get_param('~fps', 30)
        self.bridge = CvBridge()
        self.video_writer = None
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        file_name = f"video_{timestamp}.mp4"
        self.output_path = os.path.join(self.video_path, file_name)

        rospy.loginfo(f"Video will be saved to: {self.output_path}")

        # Subscribe to the camera topic
        self.image_topic = rospy.get_param('~image_topic')
        rospy.Subscriber(self.image_topic, Image, self.image_callback)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Initialize the video writer if not already done
        if self.video_writer is None:
            height, width = frame.shape[:2]
            self.video_writer = cv2.VideoWriter(
                self.output_path,
                cv2.VideoWriter_fourcc(*'mp4v'),  # Codec
                self.fps,
                (width, height)
            )

        # Write the frame to the video file
        self.video_writer.write(frame)

    def cleanup(self):
        # Release the video writer when done
        if self.video_writer:
            self.video_writer.release()
        rospy.loginfo("Video recording stopped.")

if __name__ == "__main__":
    recorder = VideoRecorder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down video recorder.")
    finally:
        recorder.cleanup()
