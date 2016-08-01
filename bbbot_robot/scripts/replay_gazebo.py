#! /usr/bin/env python
from bbbot_robot.evaluate import EvaluateGazebo
import sys
import os
import rospy


DMP = 15
BAG_FILE = '/home/nikhilkalige/workspace/ros/devel_packages/bag_files/2016-05-15-15-20-40.bag'


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
