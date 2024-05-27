import time

import pybullet as p
import pybullet_data

import numpy as np


def trajectory(t):
    '''
    A function to specify the trajectory
    Args:
    t: time
    Returns:
    x: X coordinate of the end-effector
    y: Y coordinate of the end-effector
    z: Z coordinate of the end-effector
    '''

    r = 0.4
    a = 0.05

    omega = 2 * np.pi * 2
    omega_z = 2 * np.pi * 10

    # Centre of the trajectory
    [x0, y0, z0] = [0.5, 0.5, 0.25]

    x = x0 + r * np.cos(omega * t) - r * np.sin(omega * t)
    y = y0 + r * np.sin(omega * t) + 0.5
    z = z0 + a * np.sin(omega_z * t) - r * np.sin(omega *t)

    return x, y, z


def circle_trajectory(t, T=1.0, r=3.0, z=1.0):
    '''
    A function to specify the circular trajectory
    Args:
    t: time
    Returns:
    T: periodicity of the circle
    r: circle radius
    z: Z coordinate of the points
    '''

    omega = 2 * np.pi / T

    x = + r * np.cos(omega * t)
    y = + r * np.sin(omega * t)

    return x, y, z


def show_debug_trajectory(points, color=(0.0, 1.0, 0.0), duration=20.0):
    '''
        Debugging trajectory
        Args:
        points: points to show of (n, 3) size
        color: trajectory color
        duration: the time after which the visualization will disappear, 0.0 time
        makes it permanent
    '''
    for i in range(len(points) - 1):
        p.addUserDebugLine(points[i], points[i + 1], lineColorRGB=color, lifeTime=duration)


def show_debug_points(points, color=(1.0, 0.0, 0.0), duration=1.0):
    '''
        Debugging points
        Args:
        points: points to show of (n, 3) size
        color: points color
        duration: the time after which the visualization will disappear, 0.0 time
        makes it permanent
    '''
    colors = [color for _ in range(len(points))]
    p.addUserDebugPoints(link_points, pointColorsRGB=colors, lifeTime=duration)


client = p.connect(p.GUI)  # or p.GUI

# Load the URDF of the plane that forms the ground
# p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the search path to find the plane.urdf file
# plane = p.loadURDF("plane.urdf")


# Load the URDF of the robot
dt = 0.001  # Simulation time-step
ik_solver = 0
robot_id = p.loadURDF("spatial_3R_robot.urdf")
p.resetBasePositionAndOrientation(robot_id, [0, 0, 0], [0, 0, 0, 1])
num_joints = p.getNumJoints(robot_id)
end_effector_id = num_joints - 1

####### Initial state #########

null_states = [0.0] * num_joints

for i in range(num_joints):
    p.resetJointState(robot_id, i, null_states[i])

##################################################
# Set the necessary parameters for the simulation
##################################################

# Set the Gravity vector
p.setGravity(0, 0, -9.81, physicsClientId=client)
p.setTimeStep(0.001)  # The lower this is, more accurate the simulation
p.stepSimulation()
link_state = p.getLinkState(robot_id, end_effector_id)
link_pos, link_orn = link_state[4], link_state[5]

joint_indexes = np.arange(num_joints).tolist()

current_joint_values = [x[0] for x in p.getJointStates(robot_id, joint_indexes)]

inverse_joint_values = p.calculateInverseKinematics(robot_id, end_effector_id, link_pos, restPoses=null_states)


################################################################################
# Create a Proportional control loop to regulate the position of a single joint
################################################################################

start = 0.0
end = 1.0

t = np.linspace(start, end, int((end - start) / dt))
pos_t = [trajectory(x) for x in t]
# pos_t = [circle_trajectory(x, T=5.0, r=1.5, z=1.0) for x in t]

######## Set up the robot in the target positions #########
show_debug_trajectory(pos_t)

for i in range(len(pos_t)):
    inverse_joint_values = p.calculateInverseKinematics(robot_id, end_effector_id, pos_t[i],
                                                        solver=ik_solver)

    for j in range(num_joints):
        try:

            p.resetJointState(robot_id, j, inverse_joint_values[j])

        except IndexError:
            pass

    link_states = p.getLinkStates(robot_id, joint_indexes)
    link_points = [link_states[i][4] for i in range(num_joints)]
    show_debug_points(link_points)

    p.stepSimulation()
    time.sleep(dt * 5)