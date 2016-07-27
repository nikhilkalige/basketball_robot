#! /usr/bin/env python
from bbbot_robot.cmaes import setup_cmaes, run_cmaes, checkpoint_handle, generate_name, init_creator
from bbbot_robot.cmaes_hansen import hans_setup_cmaes, hans_run_cmaes
from bbbot_robot.evaluate import Evaluate, EvaluateHansen, EvaluateGazebo, PIdx
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN
import sys
import os
import rospy


cmaes_dump = "/home/nikhilkalige/workspace/asu/thesis/cmaes_dump"
NGEN = 1000
SIGMA = 1
DMP = 15
BAG_FILE = '/home/nikhilkalige/workspace/ros/devel_packages/bag_files/2016-05-15-15-20-40.bag'


def cmaes_hansen(robot, location):
    # cmaes = hans_setup_cmaes(SIGMA, robot.get_initial_params(), constraint_func=True)

    # p = robot.get_initial_params()
    # robot.eval(tbest)
    # return
    # filename = '/home/lonewolf/workspace/asu/thesis/cmaes_dump/12_06_15_19_21/cma_00021.pkl'

    filename = '/home/nikhilkalige/workspace/asu/thesis/cmaes_dump/26_07_18_18_00/cma_00008.pkl'
    cmaes = hans_setup_cmaes(SIGMA, robot.get_initial_params(), constraint_func=True, checkpoint=True, filename=filename)
    robot.set_evals(cmaes.countevals)
    hans_run_cmaes(cmaes, robot.eval, location)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Enter folder location")
        sys.exit()

    if not os.path.exists(sys.argv[1]):
        print("Invalid path {}".format(sys.argv[1]))
        sys.exit()

    loc = sys.argv[1]
    data_file = os.path.join(loc, 'outcmaesxmean.dat')
    if not os.path.exists(data_file):
        print('outcmaesxmean.dat doesn\'t exist in {}'.format(loc))
        sys.exit()

    rospy.init_node('bot', log_level=rospy.WARN)
    robot = EvaluateGazebo(DMP, BAG_FILE, plot=False)
    rospy.loginfo("Starting replay")

    with open(data_file, 'r') as f:
        num_lines = sum(1 for line in f) - 1
        print("Iterations count: {}".format(num_lines))

    with open(data_file, 'r') as f:
        f.readline()  # skip first line
        idx = 1
        for line in f.readlines():
            print("Evalution: {}/{}".format(idx, num_lines))
            idx += 1
            params = [float(x) for x in line.split()[5:]]
            # robot.eval(params)

    robot.track.kill()
    rospy.loginfo("Finished execution")
