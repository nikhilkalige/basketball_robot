#! /usr/bin/env python
import rospy
import sys
import numpy as np
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN


def move_to_pos(robot, joint_index, end_value, inc, delay):
    robot.create_trajectory('left', joint_index, end_value, inc, delay)
    robot.create_trajectory('right', 0, 0, 0)
    robot.adjust_traj_length()
    robot.left.echo_trajectory()
    coll = robot.check_collision()
    print(coll)
    if coll:
        return
    robot.start_trajectory(1)
    robot.wait_completion()


def move_to_pos_both(robot, joint_index, end_value1, inc1, end_value2, inc2, delay):
    robot.create_trajectory('left', joint_index, end_value1, inc1, delay)
    robot.create_trajectory('right', joint_index, end_value2, inc2, delay)
    robot.adjust_traj_length()
    robot.left.echo_trajectory()
    coll = robot.check_collision()
    print(coll)
    if coll:
        return
    robot.start_trajectory(1)
    robot.wait_completion()


def collision_test_1(robot):
    delay = 0.05

    print("Case1: Collision with self")
    print("Move wrist2: 0 -> 1.5")
    move_to_pos(robot, JN.WRIST_2, 1.5, .05, delay)
    raw_input("\n Waiting: \n")

    print("Move sh lift: 0 -> 0.41")
    move_to_pos(robot, JN.SHOULDER_LIFT, 0.41, 0.05, delay)
    raw_input("\n Waiting: \n")

    print("Move elbow: 0 -> 1.4 -> No collision")
    move_to_pos(robot, JN.ELBOW, 1.4, 0.05, delay)
    raw_input("\n Waiting: \n")

    print("Move elbow: 0 -> 1.9 -> No collision")
    move_to_pos(robot, JN.ELBOW, 1.9, 0.05, delay)
    raw_input("\n Waiting: \n")

    print("Move elbow: 0 -> 1.97 -> Yes collision")
    move_to_pos(robot, JN.ELBOW, 1.97, 0.05, delay)
    raw_input("\n Waiting: \n")

    print("Move elbow: 0 -> 1.92 -> Yes collision")
    move_to_pos(robot, JN.ELBOW, 1.92, 0.05, delay)
    raw_input("\n Waiting: \n")

    print("Case2: Collsion with stand")
    print("Move elbow: 1.9 -> 1.0 -> No collision")
    move_to_pos(robot, JN.ELBOW, 1.0, -0.05, delay)
    raw_input("\n Waiting: \n")

    print("Move wrist2: 1.5 -> 1.0")
    move_to_pos(robot, JN.WRIST_2, 1, -0.05, delay)
    raw_input("\n Waiting: \n")

    print("Move sh pan: 0 -> 0.25 -> No collision")
    move_to_pos(robot, JN.SHOULDER_PAN, 0.25, 0.05, delay)
    raw_input("\n Waiting: \n")

    print("Move sh lift: 0.41 -> 0.8 -> No collision")
    move_to_pos(robot, JN.SHOULDER_LIFT, 0.8, 0.05, delay)
    raw_input("\n Waiting: \n")

    print("Move sh lift: 0.8 -> 1.05 -> No collision")
    move_to_pos(robot, JN.SHOULDER_LIFT, 1.05, 0.05, delay)
    raw_input("\n Waiting: \n")

    print("Move sh lift: 1.05 -> 1.1 -> Probable collision")
    move_to_pos(robot, JN.SHOULDER_LIFT, 1.1, 0.05, delay)
    raw_input("\n Waiting: \n")

    print("Move sh lift: 1.1 -> 1.2 -> Probable collision")
    move_to_pos(robot, JN.SHOULDER_LIFT, 1.2, 0.05, delay)
    raw_input("\n Waiting: \n")

    print("Move sh lift: 1.2 -> 1.25 -> Yes collision")
    move_to_pos(robot, JN.SHOULDER_LIFT, 1.25, 0.05, delay)
    raw_input("\n Waiting: \n")


def collision_test_2(robot):
    delay = 0.05

    print("Case1: Collision with each other")
    print("Move pan: 0 -> 0.3")
    raw_input("\n Waiting: \n")
    move_to_pos_both(robot, JN.SHOULDER_PAN, -.3, -.05, .3, .05, delay)

    print("Move w2: 0 -> .9")
    raw_input("\n Waiting: \n")
    move_to_pos_both(robot, JN.WRIST_2, .9, .05, .9, .05, delay)

    print("Move lift: 0 -> 1.5")
    raw_input("\n Waiting: \n")
    move_to_pos_both(robot, JN.SHOULDER_LIFT, 1.5, .05, 1.5, .05, delay)

    print("Move lift: 1.5 -> 1.55")
    raw_input("\n Waiting: \n")
    move_to_pos_both(robot, JN.SHOULDER_LIFT, 1.55, .05, 1.55, .05, delay)

    print("Move lift: 1.55 -> 1.62", "Collision FOUND")
    raw_input("\n Waiting: \n")
    move_to_pos_both(robot, JN.SHOULDER_LIFT, 1.62, .05, 1.62, .05, delay)


def main():
    rospy.init_node("basketball_robot")
    robot = Robot(use_prefix=True, sim=True)

    delay = 0.05
    # collision_test_1(robot)
    collision_test_2(robot)

    # robot.control_torque()
    #
    #
    # for i in xrange(4):
    #     robot.create_trajectory('left', 0, .05, 0.05, delay)
    #     robot.create_trajectory('right', 0, -.45, -0.05, delay)
    #     robot.start_trajectory(2)
    #     robot.wait_completion()

    #     robot.create_trajectory('left', 0, -.45, -0.05, delay)
    #     robot.create_trajectory('right', 0, .05, 0.05, delay)
    #     robot.start_trajectory(2)
    #     robot.wait_completion()

    # rospy.spin()


if __name__ == '__main__':
    main()
