#! /usr/bin/env python
from bbbot_robot.cmaes import setup_cmaes, run_cmaes, checkpoint_handle, generate_name, init_creator
from bbbot_robot.cmaes_hansen import hans_setup_cmaes, hans_run_cmaes
from bbbot_robot.evaluate import Evaluate, EvaluateGroups, EvaluateGazebo, PIdx
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN
import rospy
import pickle
import sys
import numpy as np


DMP = 15
BAG_FILE = '/home/nikhilkalige/workspace/ros/devel_packages/bag_files/2016-05-15-15-20-40.bag'


if __name__ == '__main__':
    rospy.init_node('bot', log_level=rospy.INFO)
    robot = EvaluateGroups(DMP, BAG_FILE, plot=True, basket=True)
    rospy.loginfo("Finished initialization")

    filename = '/home/nikhilkalige/workspace/asu/thesis/groups_dump/2/groups-fixed.pkl'

    with open(filename, 'rb') as f:
        data = pickle.load(f)

    dunk_idx = []
    for i, d in enumerate(data):
        if d[1] == 2:
            dunk_idx.append(i)

    count = 1
    for i in dunk_idx:
        print("Evalution: {}/{}".format(count, len(dunk_idx)))
        count += 1
        raw_input()
        params = data[i][0]
        # print(params, type(params))

        # plist = params.tolist()
        # dummy_points = [0] * 60
        # for idx in [3, 5, 6, 9, 11]:
        #     plist.insert(idx, dummy_points)

        # # print(plist, type(plist))
        # mod_params = np.array(plist)
        if not params.any():
            continue

        print('Reward: ', robot.run(params, True))

    robot.track.kill()
    rospy.loginfo("Finished execution of cmaes")
