from __future__ import print_function
from bbbot_robot.robot import Robot
import numpy as np
import rospy
import sys
from bbbot_robot.robot import JointNames as JN
from bbbot_robot.tracking import Tracker
import time
import rosbag
from pydmps.dmp_discrete import DMPs_discrete


'''
Parameters:
1. Timing between the start of the three joints
    Tlift_elbow = 0 sec
    Tlift_wrist = 30 * 0.1 = 3 sec
2. Start angle for pan joint
3. Start and end angle for lift joint
4. Start and end angle for elbow joint
5. Start and end angle for wrist joint
6. Weights of dmps learned from the bag file. Decided on 15 per joint angle

Constraints:
1. T should be less than 10seconds
2. Pan should be between
3. Lift should be between
4. Elbow should be between
5. Wrist should be between
6. May be we should restrict the weights between +/- 1000
7. Since the maximum speed is 0.06 rad @ 0.017 sec, (end-start)/0.06 * 0.01 should not be smaller
    than OVERALL_TIME - TLift_

Order of params
[tLiftElbow, tLiftWrist, startPan, startLift, endLift, startElbow, endElbow, startWrist, endWrist,
weightsLift, weightsElbow, weightsWrist]

Totally = 9 + 3 * 15 = 54
'''


class Evaluate(object):
    INVALID_GAZEBO_REWARD = tuple([-10000])
    COLLISION_REWARD = tuple([-10000])
    TIME_ERROR_REWARD = tuple([-10000])
    JOINT_ERROR_REWARD = tuple([-10000])

    OVERALL_TIME = 6  # seconds
    TIMESTEP = 0.01
    DEFAULT_PARAMS = [0, 3, 1.697875, 0.2946925, 0.9123826, 1.660619, 0.3703937, 0.5213307, -0.529045]

    LIFT_POINTS = [50, 105]
    ELBOW_POINTS = [45, 100]
    WRIST_POINTS = [80, 100]

    PAN_RESTRICT = [0, np.pi]
    LIFT_RESTRICT = [-np.pi, np.pi]
    ELBOW_RESTRICT = [-np.pi, np.pi]
    WRIST_RESTRICT = [-np.pi, np.pi]

    def __init__(self, dmp_count, bag_file):
        self.robot = Robot(use_prefix=True, gazebo=False)
        self.track = Tracker()
        self.init_dmp(dmp_count, bag_file)
        # self.robot.control_torque()

    def get_initial_params(self):
        weights = []
        for dmp in self.dmps:
            weights += dmp.w[0].tolist()

        return self.DEFAULT_PARAMS + weights

    def eval(self, params):
        '''Params comes from the cmaes evaluate'''
        (valid, fitness) = self.constraint(params)
        if not valid:
            return fitness

        # Pick the ball
        # Move to the initial position
        if not self.move_to_initial_position(params):
            return self.COLLISION_REWARD

        # Start the throwing process
        # get the weights from params with shape (4, dmp_count)
        trajectory = []
        for idx in range(4):
            trajectory.append(self.generate_traj_points(idx, params))
        # Insert zeros in position for Wrist 1 and Wrist 3 joints
        length = len(trajectory[0])
        trajectory.insert(3, [0] * length)
        trajectory.append([0] * length)
        self.robot.trajectory_learning(trajectory)

        retval = self.visualize()
        if retval:
            return retval

        return self.real_run()

    def constraint(self, params):
        '''Return (valid, fitness)'''
        if not self.validate_time(params[0], params[5], params[6]):
            return (False, self.TIME_ERROR_REWARD)
        if not self.validate_time(params[1], params[7], params[8]):
            return (False, self.TIME_ERROR_REWARD)
        if not self.validate_joint_limits([params[2]], self.PAN_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        if not self.validate_joint_limits([params[3], params[4]], self.LIFT_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        if not self.validate_joint_limits([params[5], params[6]], self.ELBOW_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        if not self.validate_joint_limits([params[7], params[8]], self.WRIST_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        return (True, tuple())

    def validate_time(self, start_delay, start_angle, end_angle):
        # Time is positive
        if start_delay < 0:
            return False
        run_time = self.OVERALL_TIME - start_delay
        # Defines the minimum time needed to complete the trajectory
        time_to_run = (end_angle - start_angle) * 0.01 / 0.06
        if run_time <= time_to_run:
            return False
        return True

    def validate_joint_limits(self, angles, limits):
        if not (limits[0] <= angles[0] <= limits[1]):
            return False

        if len(angles) == 2:
            if not (limits[0] <= angles[1] <= limits[1]):
                return False
        return True

    def visualize(self):
        raw_input('Enter to start rviz visualization: ')
        self.robot.visualize_trajectory()
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
            position = []
            for _, msg, t in bag.read_messages():
                position.append(np.array(msg.position))

        position = np.array(position).T

        # We need only three joints
        arm_index = np.array([1, 2, 4])
        position = position[arm_index, :]

        length, valid_points = [], []
        points_idx = [self.LIFT_POINTS, self.ELBOW_POINTS, self.WRIST_POINTS]
        for idx, arr in zip(points_idx, position):
            valid_points.append(arr[idx[0]:idx[1]])
            length.append(idx[1] - idx[0])

        self.dmps = []
        for l, joint in zip(length, valid_points):
            dmp = DMPs_discrete(dmps=1, bfs=dmp_count, dt=1.0 / l)
            dmp.imitate_path(joint)
            self.dmps.append(dmp)

    def generate_traj_points(self, joint_idx, params):
        no_points = self.OVERALL_TIME * self.TIMESTEP
        # Pan joint
        if joint_idx == 0:
            angle = params[2]
            return [angle] * no_points

        # Lift Joint
        if joint_idx == 1:
            angle = [params[3], params[4]]
            start_points = 0
            dmp_points = self.OVERALL_TIME * self.TIMESTEP
            initial_points = self.LIFT_POINTS[1] - self.LIFT_POINTS[0]

        # Elbow Joint
        if joint_idx == 2:
            angle = [params[5], params[6]]
            start_points = params[0] * self.TIMESTEP
            dmp_points = self.OVERALL_TIME * self.TIMESTEP - start_points
            initial_points = self.ELBOW_POINTS[1] - self.ELBOW_POINTS[0]

        # Wrist Joint
        if joint_idx == 3:
            angle = [params[7], params[8]]
            start_points = params[1] * self.TIMESTEP
            dmp_points = self.OVERALL_TIME * self.TIMESTEP - start_points
            initial_points = self.WRIST_POINTS[1] - self.WRIST_POINTS[0]

        points = []
        points += [angle[0] * start_points]

        weights = np.array([params[(9 + (joint_idx - 1) * 15): (9 + joint_idx * 15)]])
        tau = initial_points / dmp_points

        self.dmps[joint_idx - 1].w = weights
        self.dmps[joint_idx - 1].y0 = np.array([angle[0]])
        self.dmps[joint_idx - 1].goal = np.array([angle[1]])

        result = self.dmps[joint_idx - 1].rollout(timesteps=dmp_points, tau=tau)[0]
        points += result.tolist()

        return points

    def move_to_initial_position(self, params):
        print("-----------------------Initial Position-----------------------")
        points = [2.5, 0.75, -0.4, 0, 0, 1.58]
        self.robot.interpolate('left', points)
        points = [-2.5, 0.75, -0.4, 0, 0, 3.14]
        self.robot.interpolate('right', points)

        # print self.robot.left._goal
        # print self.robot.gazebo_left._goal
        self.robot.visualize_trajectory()
        if not self.handle_input('Running move to initial position: '):
            return False

        # print self.robot.left._goal
        # print self.robot.gazebo_left._goal
        self.robot.start_trajectory(delay=1)
        self.robot.wait_completion()

        print("-----------------------DMP Position-----------------------")

        lpoints = [params[2], params[3], params[5], 0, params[7], 1.58]
        self.robot.interpolate('left', points, skip_joints=[JN.SHOULDER_LIFT])
        rpoints = lpoints[:]
        rpoints[0] *= -1
        rpoints[5] = 3.14
        self.robot.interpolate('right', points, skip_joints=[JN.SHOULDER_LIFT])

        print("-----------------------DMP Position-----------------------")
        self.robot.start_trajectory(delay=1, gazebo=True)
        self.robot.wait_completion(gazebo=True)
        k = raw_input('Running move to initial dmp position')
        if 'n' in k:
            return False

        self.robot.start_trajectory(delay=1)
        self.robot.wait_completion()

        lpoints = [params[2], params[3], params[5], 0, params[7], 1.58]
        self.robot.interpolate('left', points)
        rpoints = lpoints[:]
        rpoints[0] *= -1
        rpoints[5] = 3.14
        self.robot.interpolate('right', points)

        print("-----------------------Ball Pickup Position-----------------------")
        self.robot.start_trajectory(delay=1, gazebo=True)
        self.robot.wait_completion(gazebo=True)
        k = raw_input('Running move to ball pickup')
        if 'n' in k:
            return False

        self.robot.start_trajectory(delay=1)
        self.robot.wait_completion()

        return True
