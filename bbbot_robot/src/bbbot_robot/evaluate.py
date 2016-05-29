from bbbot_robot.robot import Robot
import numpy as np
import rospy
import sys
from bbbot_robot.robot import JointNames as JN
from bbbot_robot.tracking import Tracker
import time
import rosbag
from pydmps.dmp_discrete import DMPs_discrete


class Evaluate(object):
    INVALID_GAZEBO_REWARD = tuple([-1])
    COLLISION_REWARD = tuple([-1])

    def __init__(self, dmp_count, bag_file):
        self.robot = Robot(use_prefix=True, gazebo=True)
        self.track = Tracker()
        self.init_dmp(dmp_count, bag_file)

    def eval(self, params):
        '''Params comes from the cmaes evaluate'''
        (valid, fitness) = self.constraint(params)
        if not valid:
            return fitness

        # get the weights from params with shape (4, dmp_count)
        dmp_weights = params['w']
        self.dmp.w = dmp_weights

        self.dmp.y0 = params['start']
        self.dmp.goal = params['end']
        gen_path, _, _ = self.dmp.rollout()

        retval = self.gazebo()
        if retval:
            return retval

        return self.real_run()

    def constraint(self, params):
        '''(valid, fitness)'''
        pass

    def gazebo(self):
        raw_input('Enter to start gazebo simulation: ')
        self.robot.start_trajectory(.1, gazebo=True)
        self.robot.wait_completion(gazebo=True)
        ans = raw_input('Was the gazebo movement valid: ')
        if 'n' in ans:
            return self.INVALID_GAZEBO_REWARD
        else:
            return tuple()

    def real_run(self):
        collision = self.robot.check_collision()
        if collision:
            return self.COLLISION_REWARD

        raw_input('Enter to start robot movement: ')

        self.track.start()
        self.robot.start_trajectory(2)
        self.robot.wait_completion()
        self.track.stop()

        reward = self.track.get_reward()
        print("Got reward:", reward)

        ans = raw_input('Is the reward valid: ')
        if 'n' in ans:
            while True:
                ans = raw_input('Enter new reward: ')
                try:
                    ans = float(ans)
                    break
                except ValueError:
                    pass

            return tuple([ans])
        else:
            return tuple([reward])

    def init_dmp(self, dmp_count, bag_file):
        with rosbag.Bag(bag_file, 'r') as bag:
            position, velocity, time_arr = [], [], []
            for _, msg, t in bag.read_messages():
                position.append(np.array(msg.position))
                velocity.append(np.array(msg.velocity))
                time_arr.append(t.to_sec())

        position = np.array(position).T
        velocity = np.array(velocity)
        time_arr = np.array(time_arr)

        accel = np.diff(velocity, axis=0)
        time_arr = np.diff(time_arr)
        accel = np.divide(accel, time_arr[0])
        accel = np.insert(accel, 0, 0, axis=0)

        arm_index = np.array([1, 2, 4])
        initial_pos, goal_pos = [], []
        for idx in arm_index:
            initial_pos.append(position[idx][0])
            goal_pos.append(position[idx][-1])

        length = position.shape[1]
        self.dmp = DMPs_discrete(dmps=arm_index.shape[0], bfs=dmp_count, dt=1.0 / length)
        self.dmp.imitate_path(position)
