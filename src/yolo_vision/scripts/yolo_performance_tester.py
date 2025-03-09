#!/usr/bin/env python3
import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import psutil
import subprocess
import os
import signal
from datetime import datetime

class YoloPerformanceTester:
    def __init__(self):
        rospy.init_node('yolo_performance_tester', anonymous=True)
        
        # Parameters
        self.implementation = rospy.get_param('~implementation', 'python')  # 'python' or 'cpp'
        self.test_duration = rospy.get_param('~test_duration', 60)  # seconds
        self.image_topic = rospy.get_param('~image_topic', '/locobot/camera/color/image_raw')
        self.annotated_topic = rospy.get_param('~annotated_topic', '/locobot/camera/yolo/annotated_image')
        
        # Metrics
        self.frame_count = 0
        self.latencies = []
        self.cpu_usages = []
        self.memory_usages = []
        self.start_time = None
        self.last_image_time = None
        self.process_id = None
        
        # Find process ID (either Python or C++)
        if self.implementation == 'python':
            self.find_process('yolo_node.py')
        else:
            self.find_process('yolo_node')
        
        # Subscribe to the original image topic to track when images are published
        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        # Subscribe to the annotated image to calculate latency
        rospy.Subscriber(self.annotated_topic, Image, self.annotated_callback)
        
        rospy.loginfo(f"Starting performance test for {self.implementation} implementation")
        rospy.loginfo(f"Monitoring process ID: {self.process_id}")
        rospy.loginfo(f"Test will run for {self.test_duration} seconds")
        
        # Start the test
        self.start_time = time.time()
        
        # Create timer to collect resource usage
        self.timer = rospy.Timer(rospy.Duration(0.1), self.collect_resource_usage)
        
        # Register shutdown hook
        rospy.on_shutdown(self.save_results)

    def find_process(self, process_name):
        """Find the process ID of the YOLO node"""
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if process_name in ' '.join(proc.info['cmdline']):
                    self.process_id = proc.info['pid']
                    return
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        
        rospy.logwarn(f"Could not find process: {process_name}")
        self.process_id = None

    def image_callback(self, msg):
        """Track when images are received"""
        self.last_image_time = time.time()

    def annotated_callback(self, msg):
        """Calculate latency when processed images are received"""
        if self.last_image_time is not None:
            latency = (time.time() - self.last_image_time) * 1000  # ms
            self.latencies.append(latency)
            self.frame_count += 1

    def collect_resource_usage(self, event):
        """Collect CPU and memory usage of the YOLO node"""
        if self.process_id is not None:
            try:
                process = psutil.Process(self.process_id)
                
                # Get CPU usage (%)
                self.cpu_usages.append(process.cpu_percent())
                
                # Get memory usage (MB)
                memory_info = process.memory_info()
                memory_mb = memory_info.rss / (1024 * 1024)
                self.memory_usages.append(memory_mb)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                rospy.logwarn("Could not access process metrics")
        
        # Check if test should end
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= self.test_duration:
            rospy.signal_shutdown("Test completed")

    def save_results(self):
        """Save and display test results"""
        # Calculate metrics
        elapsed_time = time.time() - self.start_time
        fps = self.frame_count / elapsed_time
        
        # Print results
        rospy.loginfo("Performance Test Results:")
        rospy.loginfo(f"Implementation: {self.implementation}")
        rospy.loginfo(f"Total frames processed: {self.frame_count}")
        rospy.loginfo(f"Average FPS: {fps:.2f}")
        
        if self.latencies:
            avg_latency = np.mean(self.latencies)
            min_latency = np.min(self.latencies)
            max_latency = np.max(self.latencies)
            p95_latency = np.percentile(self.latencies, 95)
            
            rospy.loginfo(f"Average latency: {avg_latency:.2f} ms")
            rospy.loginfo(f"Min latency: {min_latency:.2f} ms")
            rospy.loginfo(f"Max latency: {max_latency:.2f} ms")
            rospy.loginfo(f"95th percentile latency: {p95_latency:.2f} ms")
        
        if self.cpu_usages:
            avg_cpu = np.mean(self.cpu_usages)
            rospy.loginfo(f"Average CPU usage: {avg_cpu:.2f}%")
        
        if self.memory_usages:
            avg_memory = np.mean(self.memory_usages)
            rospy.loginfo(f"Average memory usage: {avg_memory:.2f} MB")
        
        # Save results to file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"yolo_{self.implementation}_performance_{timestamp}.txt"
        
        with open(filename, 'w') as f:
            f.write(f"YOLO Performance Test Results - {self.implementation}\n")
            f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Total frames processed: {self.frame_count}\n")
            f.write(f"Test duration: {elapsed_time:.2f} seconds\n")
            f.write(f"Average FPS: {fps:.2f}\n")
            
            if self.latencies:
                f.write(f"Average latency: {avg_latency:.2f} ms\n")
                f.write(f"Min latency: {min_latency:.2f} ms\n")
                f.write(f"Max latency: {max_latency:.2f} ms\n")
                f.write(f"95th percentile latency: {p95_latency:.2f} ms\n")
            
            if self.cpu_usages:
                f.write(f"Average CPU usage: {avg_cpu:.2f}%\n")
            
            if self.memory_usages:
                f.write(f"Average memory usage: {avg_memory:.2f} MB\n")
        
        rospy.loginfo(f"Results saved to {filename}")
        
        # Create visualization if we have enough data
        if len(self.latencies) > 10:
            self.visualize_results(timestamp)

    def visualize_results(self, timestamp):
        """Create visualizations of the performance metrics"""
        plt.figure(figsize=(12, 10))
        
        # Plot latency distribution
        plt.subplot(3, 1, 1)
        plt.hist(self.latencies, bins=30, alpha=0.7, color='blue')
        plt.axvline(np.mean(self.latencies), color='red', linestyle='dashed', linewidth=1)
        plt.title(f'{self.implementation} Implementation - Latency Distribution')
        plt.xlabel('Latency (ms)')
        plt.ylabel('Frequency')
        
        # Plot CPU usage over time
        plt.subplot(3, 1, 2)
        plt.plot(np.linspace(0, self.test_duration, len(self.cpu_usages)), self.cpu_usages, 'g-')
        plt.title('CPU Usage Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('CPU Usage (%)')
        
        # Plot memory usage over time
        plt.subplot(3, 1, 3)
        plt.plot(np.linspace(0, self.test_duration, len(self.memory_usages)), self.memory_usages, 'm-')
        plt.title('Memory Usage Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Memory Usage (MB)')
        
        plt.tight_layout()
        plt.savefig(f"yolo_{self.implementation}_performance_{timestamp}.png")
        rospy.loginfo(f"Performance visualization saved to yolo_{self.implementation}_performance_{timestamp}.png")


if __name__ == '__main__':
    try:
        tester = YoloPerformanceTester()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass