#! /usr/bin/env python
from bbbot_robot.evaluate import EvaluateHansen
from bbbot_robot.config_reader import conf
import sys
import os
import rospy


DMP = 15
BAG_FILE = '/home/nikhilkalige/workspace/ros/devel_packages/bag_files/2016-05-15-15-20-40.bag'


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Enter folder name")
        sys.exit()

    folder_name = sys.argv[1]
    folder = conf.get('dump', 'cmaes_dump')
    loc = os.path.join(folder, folder_name)

    if not os.path.exists(loc):
        print("Invalid path {}".format(loc))
        sys.exit()

    data_file = os.path.join(loc, 'outcmaesxmean.dat')
    if not os.path.exists(data_file):
        print('outcmaesxmean.dat doesn\'t exist in {}'.format(loc))
        sys.exit()

    rospy.init_node('bot')
    robot = EvaluateHansen(DMP, BAG_FILE, plot=False, basket=True)
    rospy.loginfo("Starting replay")

    with open(data_file, 'r') as f:
        num_lines = sum(1 for line in f) - 1
        print("Iterations count: {}".format(num_lines))

    with open(data_file, 'r') as f:
        f.readline()  # skip first line
        idx = 1
        for line in f.readlines():
            print("Evalution: {}/{}".format(idx, num_lines))
            raw_input()
            params = [float(x) for x in line.split()[5:]]
            robot.eval(params)
            idx += 1
