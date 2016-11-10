#! /usr/bin/env python
import rospy
import sys
import numpy as np
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN
import time


p1 = [1.9343172, 0.3035183, 1.6085280, -0.0115048, 0.5887746, -1.3481548,
      2.7264191945523018, 1.3592411379414227, -0.500013357376569, -0.012593911958158998, 1.2648478057672812, -1.9538472629239634]


def move_to_point(robot, pt):
    delay = 0.1

    for pos in xrange(6):
        robot.create_trajectory('left', pos, pt[pos], 0.05, delay)
        if not robot.single_arm:
            robot.create_trajectory('right', pos, pt[pos + 6], 0.05, delay)

        robot.start_trajectory(0.5)
        robot.wait_completion()


def move_to_pos(robot, joint, leftpos, rightpos, delay, move=True, inc=0.06):
    robot.create_trajectory('left', joint, leftpos, inc, delay)
    if not robot.single_arm:
        robot.create_trajectory('right', joint, rightpos, inc, delay)

    if move:
        robot.start_trajectory(2)
        robot.wait_completion()


def initial_position(robot):
    move_to_point(robot, p1)
    return


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
    robot = Robot(use_prefix=True, single_arm=False, sim=False)

    # robot.control_torque(False)
    robot.control_torque()
    # return

    # initial_position(robot)

    time.sleep(2)

    # move_to_pos(robot, JN.SHOULDER_PAN, 1.3, -1.3, .1, True)
    # return

    # throw(robot)
    # robot.trajectory_from_file('/home/lonewolf/workspace/ros/devel_packages/test1.txt')
    p1 = [1.6978997, 0.2941041, 1.6606187, -0.0115674, 0.5213307, -6.3115196,
          -1.7463099, 0.2575868, 1.7589914, -0.0091512, 0.4493160, -3.7258586]
    filename = '2016-05-15-15-20-40.bag'
    filename_txt = '2016-05-15-15-20-40.txt'

    p2 = [1.3955331, 0.5372943, 1.3424659, -0.0124061, 0.7577050, -3.4700225,
          -1.3353428, 0.3326495, 1.3602426, -0.0013395, 0.8814660, -3.6531200]

    p3 = [1.3, 0.2941041, 1.6606187, -0.0115674, 0.5213307, -6.3115196,
          -1.3, 0.2575868, 1.7589914, -0.0091512, 0.4493160, -3.7258586]
    # filename = '2016-05-15-16-43-39.bag'

    move_to_point(robot, p3)
    # robot.trajectory_from_bag('/home/lonewolf/workspace/ros/devel_packages/bag_files/' + filename, use_recorded_time=False, delay=0.01)
    robot.trajectory_from_file('/home/lonewolf/workspace/ros/devel_packages/bag_files/' + filename_txt, delay=0.01)

    raw_input("Validate generated angle values: \n")
    robot.start_trajectory(2)
    robot.wait_completion()


if __name__ == '__main__':
    main()
