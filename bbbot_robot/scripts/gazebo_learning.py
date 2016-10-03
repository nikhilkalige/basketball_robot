#! /usr/bin/env python
from bbbot_robot.cmaes import setup_cmaes, run_cmaes, checkpoint_handle, generate_name, init_creator
from bbbot_robot.cmaes_hansen import hans_setup_cmaes, hans_run_cmaes
from bbbot_robot.evaluate import Evaluate, EvaluateHansen, EvaluateGazebo, PIdx
from bbbot_robot.evaluate_dual import EvaluateDualGazebo
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN
import sys
import os
import rospy


cmaes_dump = "/home/nikhilkalige/workspace/asu/thesis/cmaes_dump"
NGEN = 70
SIGMA = 1
DMP = 15
BAG_FILE = '/home/nikhilkalige/workspace/ros/devel_packages/bag_files/2016-05-15-15-20-40.bag'


def cmaes_hansen(robot, location):
    cmaes = hans_setup_cmaes(SIGMA, robot.get_initial_params(), constraint_func=True)

    # p = robot.get_initial_params()
    # robot.eval(tbest)
    # return
    # filename = '/home/lonewolf/workspace/asu/thesis/cmaes_dump/12_06_15_19_21/cma_00021.pkl'

    # filename = '/home/nikhilkalige/workspace/asu/thesis/cmaes_dump/16_08_20_36_33/cma_00008.pkl'
    # cmaes = hans_setup_cmaes(SIGMA, robot.get_initial_params(), constraint_func=True, checkpoint=True, filename=filename)
    # robot.set_evals(cmaes.countevals)

    hans_run_cmaes(cmaes, robot.eval, location)


if __name__ == '__main__':
    rospy.init_node('bot', log_level=rospy.WARN)
    robot = EvaluateGazebo(DMP, BAG_FILE, plot=True, basket=True)
    #robot = EvaluateDualGazebo(DMP, BAG_FILE, plot=True, basket=True)
    rospy.loginfo("Finished initialization")

    location = generate_name(cmaes_dump)
    rospy.loginfo("Location: {}".format(location))

    cmaes_hansen(robot, location)

    robot.track.kill()
    rospy.loginfo("Finished execution of cmaes")
