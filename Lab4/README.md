# Lab4 - Creating robot's model

> [!NOTE]
> Based on [Rafał Staszak's repository](https://github.com/RafalStaszak/sample_robot)

Put the src directory inside of your ros2 workspace!

## Task1 - change structure of a robot inside [single_rrbot.urdf](/Lab4/src/sample_robot/sample_robot_bringup/launch/single_rrbot.urdf).

## Task2 - set a sequence of moves for each joint inside [sample_joint_states.py](/Lab4/src/sample_robot/sample_joints/sample_joints/sample_joint_states.py).

## Task3 - register and display state of each joint by editing [show_joint_states.py](/Lab4/src/sample_robot/sample_joints/sample_joints/show_joint_states.py).

### Results:
![Working robot](/pictures/lab4.gif)

![Joint states](/pictures/Lab4_Task3.png)

### How to run:

While you are inside your ros2 workspace (not inside the src directory) use those commands:

```colcon build```

```source install/setup.bash```

```ros2 launch sample_robot_bringup sample_robot_bringup_launch.py```

In second terminal:

```source install/setup.bash```

```rviz2```

To setup rviz2 properly you have to:
- add **TF** to display
- change **Fixed frame** to value **world**

In third terminal:

```source install/setup.bash```

```ros2 run sample_joints show_joint_states```

**Because there is a single sequence, you might need to use ```ros2 launch sample_robot_bringup sample_robot_bringup_launch.py``` more than once to see how it's running**
