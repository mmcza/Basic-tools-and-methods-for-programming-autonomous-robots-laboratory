# Lab8 - LiDAR

> [!NOTE]
> Based on [Rafał Staszak's repository](https://github.com/RafalStaszak/turtlebot3_autocontrol) and [ROBOTIS-GIT's repository](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

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
sudo apt-get install ros-humble-tf-transformations
```

```
sudo pip3 install transforms3d

```

and after that build the package:

```
colcon build --symlink-install
```

```
source install/setup.bash
```

## Task - Implement the algorithm for traversing a labyrinth with the Turtlebot3 robot in the script [traverse.py](/Lab8/src/turtlebot3_autocontrol/turtlebot3_autocontrol/traverse.py). The script should create a mechanism for guiding the robot based on information from the LiDAR sensor by subscribing to the topic /scan. After finishing at the end of the labirynth log in console position of the robot. 

### The procedure should consist of the following elements:
- Rotation in place to initially orient the robot towards the corridor.
- Traversal of the labyrinth and maintaining a collision-free trajectory.
- Procedure for checking the end of the traversal.

### To start the simulator:

```
source /usr/share/gazebo/setup.bash
```

```
export TURTLEBOT3_MODEL=burger
```

```
source install/setup.bash
```

```
ros2 launch turtlebot3_autocontrol turtlebot3_world.launch.py
```

### To start the autocontroller (in second terminal):
```
source install/setup.bash
```

```
ros2 launch turtlebot3_autocontrol traverse.launch.py
```

## Results

![gif from recording](/pictures/PNiMPRA_Lab8_LIDAR.gif)
![screenshot of position](/pictures/PNiMPRA_Lab8_Task2.png)

## About the algorithm

I divided the algorithm into 4 states:

0. Find the angle for robot to rotate - algorithm check how far are the walls in each direction and chooses the one that is the farthest from the robot ('inf' values are not counted).

1. Rotate the robot towards the labirynth - algorithm stops when orientation of the robot is close enough to the reference orientation.

2. Calculate ratio for distance between robot and left and right wall (I chose values read by LiDAR at 30 and 330 degrees) - it makes the robot turn in right direction (or move forward if it's in the middle of the corridor and there is no turn coming)

3. Stop when there are no walls next to the robot.