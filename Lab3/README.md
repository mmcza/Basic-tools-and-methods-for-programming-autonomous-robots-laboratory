# Lab3 - ROS Introduction

Put the src directory inside of your ros2 workspace!

### Results

![Task2](/pictures/Lab3_Task2.png)
![Task3](/pictures/Lab3_Task3.png)
![Task4](/pictures/Lab3_Task4.png)

## Task1 - check information about nodes and topics

Inside your ros2 workspace (not inside src) use those commands:

```colcon build --packages-select py_pubsub ```

```source install/setup.bash ```

```ros2 run py_pubsub talker```

and in second terminal

```source install/setup.bash ```

```ros2 run py_pubsub listener```

## Task2 - calculate result based on random numbers received from a subscribed topic

```colcon build --packages-select lab3_number_listener ```

```source install/setup.bash ```

```ros2 run lab3_number_listener publisher```

and in second terminal

```source install/setup.bash ```

```ros2 run lab3_number_listener subscriber```

In second terminal you will see result of a function (a1+a2+a3+a4)*(a5+a6+a7+a8) - where a8 is most recent received value.

## Task3 - set a square trajectory for turtlesim

In first terminal you're supposed to launch [turtlesim](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#use-turtlesim):

```ros2 run turtlesim turtlesim_node```

In second terminal you launch script to set trajectory for turtlesim (published as type [*geometry_msgs/Twist Message*](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html) to topic /turtle1/cmd_vel)

```colcon build --packages-select lab3_turtle_package ```

```source install/setup.bash ```

```ros2 run lab3_turtle_package talker```

#### How does it work? 

'minimal publisher' node created in python script publish a message to either go forward (3 times in a row) :
```Python
 msg.linear.x = 1.0
 ``` 
or turn left (once)

```Python
msg.angular.z = np.pi/2
```
That creates a loop made of 4 steps.

## Task4 - calculate distance traveled by turtlesim

You need type in terminal:

```colcon build --packages-select lab3_turtle_package ```

```source install/setup.bash ```

```ros2 run lab3_turtle_package distance```

In second terminal you need to run turtlesim

```ros2 run turtlesim turtlesim_node```

And to make the turtle move you should use the trajectory from Task3:

```source install/setup.bash ```

```ros2 run lab3_turtle_package talker```