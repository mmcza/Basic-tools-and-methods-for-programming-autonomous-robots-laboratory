# Lab5 - Physical robot simulation

> [!NOTE]
> Based on [Rafał Staszak's repository](https://github.com/RafalStaszak/ur_py_control)

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

I found that sometimes there are errors when there is more packages inside src directory - the solution is to remove them.

## Running simulation

To simply run rviz with the robot:
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=127.0.0.1 use_fake_hardware:=true launch_rviz:=true initial_joint_controller:=joint_trajectory_controller
```

To make it work with MoveIt:
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=127.0.0.1 use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
```

```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true use_fake_hardware:=true
```

## Task -- Create a trajectory for robot

The task was to edit [publisher_joint_trajectory_controller.py](/Lab5/src/ur_py_control/ur_py_control/publisher_joint_trajectory_controller.py) so it has a trajectory of 6 positions. Than next goal should be published when the previous goal position is reached (at the beginning it was happening every 6 seconds).


To run first you need to have it running with MoveIt and than
```
ros2 launch ur_py_control joint_trajectory_controller.launch.py
```

To get the graphs of joint states:
```
ros2 run ur_py_control show_joint_states
```

### Results:
![Working robot](/pictures/Lab5_Task3.gif)

![Joint states](/pictures/Lab5_Task3.png)