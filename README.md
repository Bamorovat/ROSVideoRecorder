# ROS Video Recorder

## Author
Mohammad Hossein Bamorovat Abadi

## Date
2024-01-11

## Overview
This ROS package provides functionality for recording video streams from various ROS topics into video files. The `VideoRecorder` class is central to this package, taking a mapping of ROS topics to output folders, and managing the video recording process for each topic.

## Key Features
- Subscribes to multiple ROS topics.
- Records video streams from these topics.
- Supports custom output folders for each topic.
- Optional debug mode for additional output information.
- Ability to display live streams during recording (if enabled).

## Parameters
- `topics_to_folders` (dict): A dictionary mapping ROS topics to their corresponding output folders.
- `output_folder` (str): Specifies the path to the general output folder where the recorded videos are saved.
- `frame_rate` (float): Sets the frame rate for the video recordings.
- `debug` (bool): If enabled, provides additional console output for troubleshooting.
- `show_images` (bool): If enabled, displays the live video feed during the recording process.


### Downloading and Building the Package
1. **Clone the Repository:**
   First, navigate to your catkin workspace's `src` directory:
   ```bash
   cd ~/catkin_ws/src
   ```
   Then, clone the repository:
   ```bash
   git clone https://github.com/your-username/ros_video_recorder.git
    ```
2. Navigate back to your catkin workspace and build the package:

    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
   
3Source the workspace to include the new package environment:
    
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

## Usage
Launch the package using a ROS launch file. You can use the default settings or specify custom parameters:

- To launch with default settings:
  ```bash
  roslaunch video_recorder video_recorder.launch
