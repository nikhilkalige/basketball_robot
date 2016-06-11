#! /usr/bin/env python
from bbbot_robot.cmaes import setup_cmaes, run_cmaes, checkpoint_handle, generate_name, init_creator
from bbbot_robot.cmaes_hansen import hans_setup_cmaes, hans_run_cmaes
from bbbot_robot.evaluate import Evaluate, EvaluateHansen
import sys
import os
import rospy


cmaes_dump = "/home/lonewolf/workspace/asu/thesis/cmaes_dump"
NGEN = 1000
SIGMA = .2
DMP = 15
BAG_FILE = '/home/lonewolf/workspace/ros/devel_packages/bag_files/2016-05-15-15-20-40.bag'


def cmaes_deap(robot, location):
    # p = robot.get_initial_params()
    # robot.eval(p)

    init_creator()
    (population, start_gen, start_child, hof, logbook, cmaes, fitness) = checkpoint_handle(NGEN)
    # folder = '/home/lonewolf/workspace/asu/thesis/cmaes_dump/08_06_11_16_03'
    # pname = 'g00006'
    # # cname = 'g00003c14'
    # cname = False
    # (population, start_gen, start_child, hof, logbook, cmaes, fitness) = checkpoint_handle(NGEN, True, folder, pname, cname)

    rospy.loginfo("Finished checpoint handle")
    (toolbox, cmaes, stats) = setup_cmaes(NGEN, SIGMA, robot.get_initial_params(), cmaes, robot.eval)

    run_cmaes([toolbox, cmaes, hof, stats, logbook, start_gen, start_child, fitness, population],
              NGEN, location, False)


def cmaes_hansen(robot, location):
    cmaes = hans_setup_cmaes(SIGMA, robot.get_initial_params(), constraint_func=True)

    # filename = '/home/lonewolf/workspace/asu/thesis/cmaes_dump/09_06_17_01_07/cma_00001.pkl'
    # cmaes = hans_setup_cmaes(SIGMA, robot.get_initial_params(), checkpoint=True, filename=filename)

    hans_run_cmaes(cmaes, robot.eval, location)


if __name__ == '__main__':
    rospy.init_node('bot')
    # robot = Evaluate(DMP, BAG_FILE)
    robot = EvaluateHansen(DMP, BAG_FILE)
    rospy.loginfo("Finished initialization")

    location = generate_name(cmaes_dump)
    rospy.loginfo("Location: {}".format(location))

    # cmaes_deap(robot, location)
    cmaes_hansen(robot, location)

    robot.track.kill()
    rospy.loginfo("Finished execution of cmaes")
