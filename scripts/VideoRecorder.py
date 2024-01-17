#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Author: Mohammad Hossein Bamorovat Abadi
Date: 2024-01-11

Description: This script subscribes to a list of ROS topics and records their images as videos.
VideoRecorder class is the main class of this script. It takes a dictionary of topics and their corresponding folders
as input. The dictionary should be in the following format:
topics_to_folders = {
    "/topic1": "folder1",
    "/topic2": "folder2",
    ... }
The script creates a video writer for each topic and writes the images of that topic to the corresponding video file.
The video files are saved in the output folder.

Args:
    topics_to_folders (dict): A dictionary of topics and their corresponding folders.
    output_folder (str): The path to the output folder.
    frame_rate (float): The frame rate of the videos.
    debug (bool): If True, the script prints debugging information.
    show_images (bool): If True, the script shows the images while recording.

Usage:
    roslaunch video_recorder video_recorder.launch
    or
    roslauch video_recorder video_recorder.launch bag_path:=/path/to/bag/file.bag output_folder:=/path/to/output/folder

"""

import os
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rospkg import RosPack
import subprocess


class VideoRecorder:
    def __init__(self, topics_to_folders, output_folder='output', frame_rate=30.0, debug=True, show_images=False):
        # self.folder = None
        self.topics_to_folders = topics_to_folders
        self.show_images = show_images
        self.bridge = CvBridge()
        self.frame_rate = frame_rate
        self.video_writers = {}
        self.frame_sizes = {}
        self.debug = debug
        self.folder = {}
        self.frame_counters = {}  # Counter for each topic
        self.output_folder = output_folder

        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)

        for topic, folder in self.topics_to_folders.items():
            output_path = os.path.join(self.output_folder, folder)
            if not os.path.exists(output_path):
                os.makedirs(output_path)

            self.video_writers[topic] = {
                'writer': None,
                'output_path': output_path
            }

            if self.debug:
                print(f'Subscribing to topic {topic}')
                print(f'Output path for topic {topic}: {output_path}')

            # Initialize counter and frame size for each topic
            self.frame_counters[topic] = 0
            self.frame_sizes[topic] = None, None
            self.folder[topic] = folder

            if self.debug:
                print(f'The folder of the topic {topic} is {self.folder[topic]}')

            rospy.Subscriber(topic, Image, self.create_callback(topic))

    def init_video_writer(self, topic, frame_size):

        print(f'init_video_writer for topic {topic} ... ')

        video_file = os.path.join(self.video_writers[topic]['output_path'], self.folder[topic] + '.avi')

        if self.debug:
            print(f'Video file for topic {topic}: {video_file}')

        self.video_writers[topic]['writer'] = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'MJPG'),
                                                              self.frame_rate, frame_size)

        if self.debug:
            print(f'Initialized video writer for topic {topic} with frame size {frame_size}')

        print(f'init_video_writer for topic {topic} ... Done! ')

    def create_callback(self, topic):

        print(f'create_callback for topic {topic} ... ')

        def callback(msg):

            try:
                # Try converting ROS image message to cv2 image
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            except Exception as e:
                rospy.logerr(f"Error converting image message to cv2 on topic {topic}: {e}")
                return

            if self.frame_counters[topic] == 0:

                if self.debug:
                    print(f'frame_counters[{topic}] = {self.frame_counters[topic]}')
                    print('Getting frame size ... ')

                h, w, _ = frame.shape
                self.frame_sizes[topic] = (w, h)
                self.init_video_writer(topic, (w, h))
                self.frame_counters[topic] += 1  # Increment counter after first frame

                if self.debug:
                    print(f'Frame size for topic {topic}:', frame.shape)
                    print(f'frame_counters[{topic}] = {self.frame_counters[topic]}')

            if self.debug and self.frame_counters.get(topic, 0) == 0:
                print(f'Frame size for topic {topic}:', frame.shape)
                self.frame_counters[topic] = 1
                cv2.imshow(topic.replace('/', '_'), frame)
                cv2.waitKey(1)

            if self.video_writers[topic]['writer'] is not None:
                self.video_writers[topic]['writer'].write(frame)

                if self.debug and self.frame_counters[topic] == 1:
                    self.frame_counters[topic] += 1
                    print(f'Writing frame for topic {topic}')  # Debugging print

            if self.show_images:
                cv2.imshow(self.folder[topic], frame)
                cv2.waitKey(1)

        return callback

    def play_rosbag(self, bag_path):
        try:
            print(f"Playing ROS bag: {bag_path}")
            subprocess.run(["rosbag", "play", bag_path], check=True)
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"An error occurred while playing ROS bag: {e}")

        self.release_videos()

    def release_videos(self):
        print(f'Releasing videos ... ')

        for writer_info in self.video_writers.values():
            if writer_info['writer'] is not None:
                writer_info['writer'].release()


def main():
    rospy.init_node('video_recorder_node', anonymous=True)

    topics_to_folders = {
        "/head_camera/rgb/image_raw": "RobotView",
        "/rgbd_livingroom/rgb/image_raw": "FrontView",
        # Add more topics and their corresponding folders here
    }

    output_folder = rospy.get_param('~output_folder', '')  # Default is 'default_output' if not provided
    rosbag_path = rospy.get_param('~rosbag_path', '')
    debug = rospy.get_param('~debug', False)  # Default is False if not provided
    show_images = rospy.get_param('~show_images', False)  # Default is False if not provided
    frame_rate = rospy.get_param('~frame_rate', 20)  # Default is 20 if not provided

    if debug:

        print(f'output_folder: {output_folder}')
        print(f'rosbag_path: {rosbag_path}')
        print(f'debug: {debug}')
        print(f'frame_rate: {frame_rate}')
        print(f'show_images: {show_images}')

    video_recorder = VideoRecorder(topics_to_folders, output_folder, frame_rate=frame_rate, debug=debug,
                                   show_images=show_images)
    video_recorder.play_rosbag(rosbag_path)


if __name__ == "__main__":
    main()
