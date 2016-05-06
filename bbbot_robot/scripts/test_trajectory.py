#! /usr/bin/env python
import rospy
import sys
import numpy as np
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN


def main():
    rospy.init_node("basketball_robot")
    robot = Robot(use_prefix=True)

    delay = 0.2

    robot.control_torque(False)
    # for i in xrange(2):
    #     robot.create_trajectory('left', JN.SHOULDER_PAN, .05, 0.05, delay)
    #     robot.create_trajectory('right', JN.SHOULDER_PAN, -.45, -0.05, delay)
    #     robot.start_trajectory(2)
    #     robot.wait_completion()

    #     robot.create_trajectory('left', JN.SHOULDER_PAN, -.45, -0.05, delay)
    #     robot.create_trajectory('right', JN.SHOULDER_PAN, .05, 0.05, delay)
    #     robot.start_trajectory(2)
    #     robot.wait_completion()

    rospy.spin()


if __name__ == '__main__':
    main()
