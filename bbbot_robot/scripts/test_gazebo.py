#! /usr/bin/env python
import rospy
import sys
import numpy as np
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN


PICKUP_POS = [2.9390760359372385, 1.217528331414226, -0.17592668464016736, 0, 1.2410842115944702]
DMP = ['1.6979', '0.2947', '1.6606', '0.0000', '0.5213', '1.5800']


def main():
    robot = Robot(use_prefix=True, display=False, collision=False, sim=True)
    points = [2.5, 0.75, -0.4, 0, 0, 1.58]
    robot.interpolate('left', points)
    points = [-2.5, 0.75, -0.4, 0, 0, 3.14]
    robot.interpolate('right', points)

    robot.start_trajectory(delay=1)
    robot.wait_completion()
    raw_input()

    lpoints = PICKUP_POS + [1.58]
    robot.interpolate('left', lpoints, skip_joints=[JN.SHOULDER_LIFT])

    rpoints = lpoints[:]
    rpoints[0] *= -1
    rpoints[5] = 3.14
    robot.interpolate('right', rpoints, skip_joints=[JN.SHOULDER_LIFT])

    robot.start_trajectory(delay=1)
    robot.wait_completion()

    robot.interpolate('left', lpoints)
    robot.interpolate('right', rpoints)

    robot.start_trajectory(delay=1)
    robot.wait_completion()

    raw_input()

    lpoints = DMP
    robot.interpolate('left', lpoints)

    rpoints = lpoints[:]
    rpoints[0] *= -1
    rpoints[5] = 3.14
    robot.interpolate('right', rpoints)

    robot.start_trajectory(delay=1)
    robot.wait_completion()


if __name__ == '__main__':
    rospy.init_node("gazebo_test")
    main()
