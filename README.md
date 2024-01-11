# ROS Video Recorder

## Overview
This ROS package provides a tool to record video streams from specified ROS topics. It is particularly useful for recording data from multiple camera feeds in robotics applications. The package also includes functionality to play and record video from ROS bag files.

## Features
- Record video streams from multiple ROS topics.
- Save recorded videos in specified folders.
- Play and record video data from ROS bag files.
- Debug mode to display live video streams and print frame details.

## Prerequisites
- ROS (Robot Operating System) [Noetic/other version as applicable]
- Python 2 or Python 3
- OpenCV
- cv_bridge

## Installation
1. Navigate to your catkin workspace's source directory (e.g., `~/catkin_ws/src`).
2. Clone this repository into the source directory.
3. Navigate back to your catkin workspace and build the package:

    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
   
4. Source the workspace to include the new package environment:
    
        ```bash
        source ~/catkin_ws/devel/setup.bash
        ```

## Usage

1. **Initialization:** Launch the ROS node with the `VideoRecorder` class. Specify the topics and output folders in a dictionary format.

Example:
```python
topics_to_folders = {
    "/head_camera/rgb/image_raw": "RobotView",
    "/rgbd_livingroom/rgb/image_raw": "FrontView",
    "/omni_livingroom/image_raw": "OmniView"
}
video_recorder = VideoRecorder(topics_to_folders, debug=True)
```

1. Recording: To start recording, simply run the node. The recorder will save videos in the specified folders for each topic.

2. Playing ROS Bags: To play a ROS bag and record its topics, use the play_rosbag method.

Example:
```python
video_recorder.play_rosbag('/path/to/your/rosbag.bag')
```


