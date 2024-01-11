#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rospkg import RosPack


class VideoRecorder:
    def __init__(self, topics_to_folders, frame_rate=30.0, debug=True):
        self.topics_to_folders = topics_to_folders
        self.bridge = CvBridge()
        self.frame_rate = frame_rate
        self.video_writers = {}
        self.frame_sizes = {}
        self.debug = debug
        self.frame_counters = {}  # Counter for each topic

        ros_pack = RosPack()
        package_path = ros_pack.get_path('video_recorder')

        for topic, folder in self.topics_to_folders.items():
            output_path = os.path.join(package_path, folder)
            if not os.path.exists(output_path):
                os.makedirs(output_path)

            self.video_writers[topic] = {
                'writer': None,
                'output_path': output_path
            }

            rospy.Subscriber(topic, Image, self.create_callback(topic))

    def create_callback(self, topic):
        def callback(msg):
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if topic not in self.frame_sizes:
                h, w, _ = frame.shape
                self.frame_sizes[topic] = (w, h)
                self.init_video_writer(topic, (w, h))

            if self.debug:
                if self.frame_counters[topic] == 0:
                    print(f'Frame size for topic {topic}:', frame.shape)
                    self.frame_counters[topic] += 1  # Increment counter after first frame
                cv2.imshow(topic.replace('/', '_'), frame)
                cv2.waitKey(1)

            if self.video_writers[topic]['writer'] is not None:
                self.video_writers[topic]['writer'].write(frame)

        return callback

    def init_video_writer(self, topic, frame_size):
        video_file = os.path.join(self.video_writers[topic]['output_path'], topic.replace('/', '_') + '.avi')
        self.video_writers[topic]['writer'] = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'XVID'),
                                                              self.frame_rate, frame_size)

    def release_videos(self):
        for writer_info in self.video_writers.values():
            if writer_info['writer'] is not None:
                writer_info['writer'].release()


if __name__ == "__main__":
    rospy.init_node('video_recorder', anonymous=True)

    topics_to_folders = {
        "/head_camera/rgb/image_raw": "RobotView",
        "/rgbd_livingroom/rgb/image_raw": "FrontView",
        "/omni_livingroom/image_raw": "OmniView"
        # Add more topics and their corresponding folders here
    }

    video_recorder = VideoRecorder(topics_to_folders, debug=True)
    rospy.spin()
    video_recorder.release_videos()
