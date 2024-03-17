# Lab3 - ROS Introduction

Put the src directory inside of your ros2 workspace!

## Task1 - checking information about nodes and topics

Inside your ros2 workspace (not inside src) use those commands:

```colcon build --packages-select py_pubsub ```
```source install/setup.bash ```
```ros2 run py_pubsub talker```

and in second terminal
```source install/setup.bash ```
```ros2 run py_pubsub listener```

## Task2 - calculating result based on random numbers received from a subscribed topic

```colcon build --packages-select lab3_number_listener ```
```source install/setup.bash ```
```ros2 run lab3_number_listener publisher```

and in second terminal
```source install/setup.bash ```
```ros2 run lab3_number_listener subscriber```

## Task3

## Task4