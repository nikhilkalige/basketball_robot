#! /usr/bin/env python
import rospy
import sys
import numpy as np
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN
import time


p1 = [-0.22, 0.82, -0.341, 0, 1.34, -1.5,
      0.22, 0.82, -0.341, 0, 1.34, -1.5]

p2 = [-0.22, 1.256, -0.341, 0, 1.34, -1.5,
      0.22, 1.256, -0.341, 0, 1.34, -1.5]

p3 = [-2.66, 1.3398, -0.44540, -0.01134, 1.22350, -1.46316,
      2.66, 1.3844, -0.5, 0.12492, 1.3063, -1.883]

p4 = [-2.7108958835899584, 1.3141233857573222, -0.37789247153974903, -0.011454701234309623, 1.20966828509447, -1.463353442730415,
      2.7264191945523018, 1.3592411379414227, -0.500013357376569, -0.012593911958158998, 1.2648478057672812, -1.9538472629239634]

# For shoulder and elbow tucked in
p5 = [-2.35, 0.22, 1.24, -0.01, 0.69, -1.45,
      2.7264191945523018, 1.3592411379414227, -0.500013357376569, -0.012593911958158998, 1.2648478057672812, -1.9538472629239634]

# For shoulder and elbow straight
p6 = [-2.35, 1.57, -.80, -0.01, 1.32, -1.45,
      2.7264191945523018, 1.3592411379414227, -0.500013357376569, -0.012593911958158998, 1.2648478057672812, -1.9538472629239634]

# For Sling Shot
p7 = [0, -1.55, -.85, -0.01, 0.75, -1.88,
      2.7264191945523018, 1.3592411379414227, -0.500013357376569, -0.012593911958158998, 1.2648478057672812, -1.9538472629239634]


def move_to_point(robot, pt):
    delay = 0.2

    for pos in xrange(6):
        robot.create_trajectory('left', pos, pt[pos], 0.05, delay)
        if not robot.single_arm:
            robot.create_trajectory('right', pos, pt[pos + 6], 0.05, delay)

        robot.start_trajectory(2)
        robot.wait_completion()


def move_to_pos(robot, joint, leftpos, rightpos, delay, move=True, inc=0.06):
    robot.create_trajectory('left', joint, leftpos, inc, delay)
    if not robot.single_arm:
        robot.create_trajectory('right', joint, rightpos, inc, delay)

    if move:
        robot.start_trajectory(2)
        robot.wait_completion()


def mod_trajectory_release(robot):
    # release_angle = 15
    release_angle = 12
    over_iters = 2
    ra_radian = np.deg2rad(release_angle)
    ra_per_arm = ra_radian / 2

    increment = ra_per_arm / over_iters

    length = len(robot.left._goal.trajectory.points)
    start_index = length - over_iters
    start_value = robot.left._goal.trajectory.points[start_index].positions[JN.ELBOW]
    angles = np.linspace(float(start_value), start_value - ra_per_arm, num=over_iters + 1)
    robot.left._goal.trajectory.points[start_index].positions[JN.ELBOW] = angles[1]
    start_index = start_index + 1
    robot.left._goal.trajectory.points[start_index].positions[JN.ELBOW] = angles[2]
    start_index = start_index + 1

    length = len(robot.right._goal.trajectory.points)
    start_index = length - over_iters
    start_value = robot.right._goal.trajectory.points[start_index].positions[JN.ELBOW]
    angles = np.linspace(float(start_value), start_value - ra_per_arm, num=over_iters + 1)
    robot.right._goal.trajectory.points[start_index].positions[JN.ELBOW] = angles[1]
    start_index = start_index + 1
    robot.right._goal.trajectory.points[start_index].positions[JN.ELBOW] = angles[2]
    start_index = start_index + 1

    # robot.left.echo_trajectory()
    # robot.right.echo_trajectory()


def initial_position(robot):
    # move_to_point(robot, p5)
    # move_to_point(robot, p6)

    # Related to sling shot
    move_to_point(robot, p7)
    return
    pt = -1.55
    move_to_pos(robot, JN.SHOULDER_LIFT, pt, -pt, .09)
    pt = -0.85
    move_to_pos(robot, JN.ELBOW, pt, -pt, .09)
    return

    pt = -2.35
    move_to_pos(robot, JN.SHOULDER_PAN, pt, -pt, .05)
    return
    pt = 1.24
    move_to_pos(robot, JN.ELBOW, pt, pt, 0.05)


def throw(robot):
    state = 4

    if state == 1:
        nextpt = 0.4
        move_to_pos(robot, JN.SHOULDER_PAN, nextpt, -nextpt, .009, False)
        position = [
            {JN.ELBOW: 71},
            {JN.SHOULDER_PAN: 0, JN.ELBOW: 51}
        ]
        robot.modify_trajectory('left', JN.SHOULDER_PAN, position)
        robot.start_trajectory(2)
        robot.wait_completion()

    elif state == 2:
        nextpt = 0.8
        move_to_pos(robot, JN.SHOULDER_PAN, nextpt, -nextpt, .009, True)

    elif state == 3:
        nextpt = 1
        move_to_pos(robot, JN.SHOULDER_LIFT, nextpt, -nextpt, .009, False)
        position = [
            {JN.ELBOW: -48.7},
            {JN.SHOULDER_LIFT: 57.3, JN.ELBOW: 32.66}
        ]
        robot.modify_trajectory('left', JN.SHOULDER_LIFT, position)
        robot.start_trajectory(2)
        robot.wait_completion()

    elif state == 4:
        nextpt = 1
        move_to_pos(robot, JN.SHOULDER_LIFT, nextpt, -nextpt, .01, False)

        # Keep the wrist upright
        position = [
            {JN.SHOULDER_LIFT: -88, JN.WRIST_2: 42.97},
            {JN.SHOULDER_LIFT: 8.07, JN.WRIST_2: -85}
        ]
        robot.modify_trajectory('left', JN.SHOULDER_LIFT, position)

        position = [
            {JN.ELBOW: -48.7},
            {JN.SHOULDER_LIFT: 57.3, JN.ELBOW: 32.66}
        ]
        robot.modify_trajectory('left', JN.SHOULDER_LIFT, position)

        position = [
            {JN.SHOULDER_LIFT: 30, JN.WRIST_2: -85},
            {JN.SHOULDER_LIFT: 57.3, JN.WRIST_2: -60}
        ]
        robot.modify_trajectory('left', JN.SHOULDER_LIFT, position)
        validate = raw_input("Validate generated angle values: \n")
        robot.start_trajectory(2)
        robot.wait_completion()


def main():
    rospy.init_node("basketball_robot")
    robot = Robot(use_prefix=True, single_arm=True, sim=True)

    # robot.control_torque()

    initial_position(robot)

    time.sleep(2)

    string = 'l'
    if string == 'l':
        nextpt = 0.35
        move_to_pos(robot, JN.SHOULDER_PAN, nextpt, -nextpt, .5, True)
    elif string == 'r':
        nextpt = -0.25
        move_to_pos(robot, JN.SHOULDER_PAN, nextpt, -nextpt, .5, True)

    time.sleep(2)
    throw(robot)


if __name__ == '__main__':
    main()
