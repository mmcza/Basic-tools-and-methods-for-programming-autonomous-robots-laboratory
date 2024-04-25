# Lab7 - ROS Rosbag and Tensorflow

> [!NOTE]
> Based on [Rafał Staszak's repository](https://github.com/RafalStaszak/ros2_object_detection)

Put the src directory inside of your ros2 workspace!

> [!NOTE]
> Before running files from tasks you need to install the following libraries and dependencies

```
apt update
```

```
apt install python3-pip
```

```
pip3 install tensorflow
```

```
pip3 uninstall protobuf
```

```
pip3 install protobuf==3.20.0
```

```
pip3 install tensorflow-object-detection-api
```

```
pip3 install opencv-python
```

and after that build the package:

```
cd /path/to/your/ros2_ws/
```

```
colcon build --symlink-install
```

```
source install/setup.bash
```

> [!NOTE]
> I cannot publish the videos and model that was used because of copyright (and rosbag is too big to upload it on Github)

## Task1 - Record bags with topics `/traffic/image`, `/traffic/image_annotated` and `/traffic/bounding_box`.

### To record a rosbag:

```
ros2 bag record -o rosbag_name /traffic/image_annotated /traffic/image /traffic/bounding_box
```

## Task2 - Play the recorded bag in a loop and use it for next tasks.

### To complete:

```
ros2 bag play rosbag_name/ --loop
```

> [!NOTE]
> You have to use the commands from each task in separate terminal (make sure to source the workspace in each of the terminals).

## Task3 - Complete the script [show_on_pcl.py](/Lab7/src/ros2_object_detection/ros_object_detection/ros_object_detection/show_on_pcl.py) in such a way, that based on the detected bounding boxes, the point cloud is modified by highlighting the objects found on the video (change the value on z axis to 0.5). Publish the modified point cloud and display it in rviz.

### To run:
```
ros2 launch ros_object_detection show_on_pcl_launch.py
```

## Task4 - Complete the script [show_tfs.py](/Lab7/src/ros2_object_detection/ros_object_detection/ros_object_detection/show_tfs.py) in such a way, that based on the detected objects, define coordinate frames attached to the center of the bounding box. Each of the TF coordinate frames should include the name in the format [object_class]_[detected_object_number]. All coordinate frames should be expressed in the map frame.

### To run:
```
ros2 launch ros_object_detection show_tfs_launch.py
```

## Task5 - Create a separate script called [show_trajectory.py](/Lab7/src/ros2_object_detection/ros_object_detection/ros_object_detection/show_trajectory.py), which will publish a message of type `sensor_msgs/Image` under the topic `/traffic/trajectory`. This message will contain a history of movement of objects in the scene. Whenever an object appears in the scene, the center of its bounding box will be marked with the appropriate color. The class "car" should be marked in blue, "pedestrians" in green, and "bicycle" in red. The output image should have a resolution of 100x100.

### To run:
```
ros2 launch ros_object_detection show_trajectory_launch.py
```

## Results

![gif from recording1](/pictures/PNiMPRA_Lab7_gif1.gif)
![gif from recording2](/pictures/PNiMPRA_Lab7_gif2.gif)

\* gifs were sped up 5x

### Setting up RVIZ

Add the following topics

![rviz settings](/pictures/PNiMPRA_Lab7_RVIZ_settings.png)