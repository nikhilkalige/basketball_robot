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


def move_to_point(robot, pt):
    delay = 0.2

    for pos in xrange(6):
        robot.create_trajectory('left', pos, pt[pos], 0.05, delay)
        robot.create_trajectory('right', pos, pt[pos + 6], 0.05, delay)
        robot.start_trajectory(2)
        robot.wait_completion()


def move_to_pos(robot, joint, leftpos, rightpos, delay, move=True, inc=0.06):
    robot.create_trajectory('left', joint, leftpos, inc, delay)
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


def main():
    rospy.init_node("basketball_robot")
    robot = Robot(use_prefix=True)

    delay = 0.2

    # robot.control_torque()
    # Move to initial position
    # move_to_point(robot, p4)

    # move_to_pos(robot, JN.ELBOW, -0.37, -0.5, 0.1)

    # Close hands
    # move_to_pos(robot, JN.SHOULDER_LIFT, 1.256, 1.256, delay)

    # Throw ball
    # nextpt = -0.341
    # move_to_pos(robot, JN.ELBOW, nextpt, nextpt, delay)

    # nextpt = 1.8
    # move_to_pos(robot, JN.SHOULDER_PAN, -nextpt, nextpt, delay)
    # move_to_pos(robot, JN.SHOULDER_PAN, -nextpt, nextpt, 0.05)
    # time.sleep(2)
    # move_to_pos(robot, JN.ELBOW, -0.37789247153974903, -0.500013357376569, delay)
    # time.sleep(2)
    # # nextpt = 0.3
    # move_to_pos(robot, JN.SHOULDER_PAN, -nextpt, nextpt, delay)
    # return

    robot.control_torque()
    # Move Faster now
    # nextpt = 0.261
    nextpt = 0.4
    move_to_pos(robot, JN.SHOULDER_PAN, nextpt, -nextpt, .009, False)
    mod_trajectory_release(robot)
    robot.start_trajectory(2)
    robot.wait_completion()

    time.sleep(2)
    nextpt = 1.8
    # move_to_pos(robot, JN.SHOULDER_PAN, -nextpt, nextpt, delay)
    move_to_pos(robot, JN.SHOULDER_PAN, -nextpt, nextpt, 0.05)
    time.sleep(2)
    move_to_pos(robot, JN.ELBOW, -0.37789247153974903, -0.500013357376569, delay)

    # Back to start

    # move_to_pos(robot, JN.SHOULDER_PAN, -2.713, 2.6436, delay)
    # nextpt = -0.341
    # move_to_pos(robot, JN.ELBOW, nextpt, nextpt, delay)

    # Open hands
    # move_to_pos(robot, JN.SHOULDER_LIFT, 0.82, 0.82, delay)


if __name__ == '__main__':
    main()
