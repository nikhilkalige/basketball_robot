#! /usr/bin/env python
from bbbot_robot.cmaes import setup_cmaes, run_cmaes, checkpoint_handle, generate_name, init_creator
from bbbot_robot.cmaes_hansen import hans_setup_cmaes, hans_run_cmaes
from bbbot_robot.evaluate import Evaluate, EvaluateHansen
import sys
import os
import rospy


cmaes_dump = "/home/lonewolf/workspace/asu/thesis/cmaes_dump"
NGEN = 1000
SIGMA = .7
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


tbest = [  1.8284905472974833, 3.8234043731516563, 4.4687962002694936, 5.9093902481269245, 6.3621251606771105, 7.1364029283308801, 3.7337401286852314, 5.8413234871325308, 3.5789461685557429, 17.688036571316168, 16.531423892854548, 15.931914016162819, 19.690284533171553, 15.524394584369295, 19.381155271817349, 18.509201347466515, 17.555922115366588, 18.959110688528355, 20.531785886779485, 18.389557344497362, 23.003878789659257, 18.145402218726701, 20.743144623815809, 21.402787391827779, 18.130823505093915, 17.401271236647492, 17.324411851191957, 16.950671791381737, 15.724880455457042, 16.89560499566506, 16.038575485729545, 14.860801025435059, 18.048539319969887, 15.983098143750244, 19.637839665936362, 19.895032080005606, 19.571717513041257, 22.128638479122625, 21.36623151735505, 16.960825149789677, 16.461194485926239, 17.313116268705937, 15.438493399683432, 15.389238189216485, 14.50499980753891, 15.903906490062029, 16.451954566493306, 18.473493479123363, 17.080038548510114, 18.886396942571647, 20.884414857503216, 20.343970031654514, 19.622613628033911, 19.193384997149437]


def cmaes_hansen(robot, location):
    # cmaes = hans_setup_cmaes(SIGMA, robot.get_initial_params(), constraint_func=True)
    # p = robot.get_initial_params()
    robot.eval(tbest)
    return
    filename = '/home/lonewolf/workspace/asu/thesis/cmaes_dump/12_06_15_19_21/cma_00021.pkl'
    cmaes = hans_setup_cmaes(SIGMA, robot.get_initial_params(), constraint_func=True, checkpoint=True, filename=filename)

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
