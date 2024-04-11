# Lab6 - Inverse Kinematics

> [!NOTE]
> Based on [Dominik Belter's repository](https://github.com/dominikbelter/ros2_ur_moveit_examples)

More information about dependencies and problems with their instalation is mentioned at the end of this file.

Put the src directory inside of your ros2 workspace!

> [!NOTE]
> Before running files from tasks you need to initialize the robot and Moveit. Do it with following commands

```
source install/setup.bash
```

```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
```

```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true use_fake_hardware:=true
```

## Task1 - Change the goal point in [hello_moveit.cpp](/Lab6/src/ROS2_ur_moveit_examples/src/hello_moveit.cpp) and watch the robot planning the path to it.

### To run:
```
ros2 launch ros2_ur_moveit_examples hello_moveit.launch.py
```

## Task2 - add 2 more obstacles inside [planning_scene.cpp](/Lab6/src/ROS2_ur_moveit_examples/src/planning_scene.cpp) and watch the robot planning the path and avoiding the obstacles.
### To run:
```
ros2 launch ros2_ur_moveit_examples planning_scene.launch.py
```


## Task3 - edit joint configuration inside [kinematics.cpp](/Lab6/src/ROS2_ur_moveit_examples/src/kinematics.cpp).
### To run:
```
ros2 launch ros2_ur_moveit_examples kinematics.launch.py
```


## Results:
### Task 1
![Robot planning the path and executing it](/pictures/PNiMPRA_Lab6_Task1.gif)

### Task 2
![Robot planning the path to avoid obstacles and executing it](/pictures/PNiMPRA_Lab6_Task2.gif)

### Task 3
![Inverse kinematics of the robot](/pictures/PNiMPRA_Lab6_Task3.png)

## Installing dependencies

First inside your ros2 workspace create a src directory. Than inside the src directory:

```
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git 
```

```
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
```

```
rosdep update
```

```
rosdep install --ignore-src --from-paths src -y
```

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```
source install/setup.bash
```

Than put the directory from inside this repository inside the src directory and when back to your ros2 workspace:

```
colcon build --packages-select ur_py_control
```

```
source install/setup.bash
```

Dependencies: ```apt-get install python3-pip``` and ```apt-get install python3-tk```

And only after that you should download the files from this repository. Than simply use ```colcon build``` in your ros2 workspace.

> [!NOTE]
> There might be issues when installing the packages. However running it 2 or 3 times solves the problem.
