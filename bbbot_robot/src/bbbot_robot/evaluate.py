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
from enum import IntEnum


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


class PIdx(IntEnum):
    D_ELBOW = 0
    D_WRIST = 1
    STR_PAN = 2
    STR_LFT = 3
    END_LFT = 4
    STR_ELB = 5
    END_ELB = 6
    STR_WRI = 7
    END_WRI = 8
    DMP_STR = 9


class ArmState(IntEnum):
    INITIAL = 1
    DMP = 2


class Evaluate(object):
    INVALID_GAZEBO_REWARD = tuple([-10000])
    COLLISION_REWARD = tuple([-10000])
    TIME_ERROR_REWARD = tuple([-10000])
    JOINT_ERROR_REWARD = tuple([-10000])

    OVERALL_TIME = 0.6  # seconds
    TIMESTEP = 0.01
    DEFAULT_PARAMS = [0, 0.3, 1.697875, 0.2946925, 0.9123826, 1.660619, 0.3703937, 0.5213307, -0.529045]

    LIFT_POINTS = [50, 105]
    ELBOW_POINTS = [45, 100]
    WRIST_POINTS = [80, 100]

    PAN_RESTRICT = [0, np.pi]
    LIFT_RESTRICT = [-np.pi, np.pi]
    ELBOW_RESTRICT = [-np.pi, np.pi]
    WRIST_RESTRICT = [-np.pi, np.pi]

    BALL_GRASP_DISTANCE = [0.41, 0.56]

    PICKUP_POS = [2.9390760359372385, 1.217528331414226, -0.17592668464016736, 0, 1.2410842115944702]

    def __init__(self, dmp_count, bag_file):
        self.robot = Robot(use_prefix=True, display=True)
        self.track = Tracker()
        self.init_dmp(dmp_count, bag_file)
        self.robot.control_torque()
        self.arm_state = ArmState.DMP

    def get_initial_params(self):
        weights = []
        for dmp in self.dmps:
            weights += dmp.w[0].tolist()

        return self.DEFAULT_PARAMS + weights

    def eval(self, params):
        """Params comes from the cmaes evaluate"""
        (valid, fitness) = self.constraint(params)
        if not valid:
            rospy.logwarn("Constraint failed")
            return fitness

        # Pick the ball
        # Move to the initial position
        if not self.move_to_initial_position(params):
            return tuple([r * 0.8 for r in self.COLLISION_REWARD])

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

    def collision_constraint(self, params):
        """ Will check for two collsions, ball pick to dmp and dmp to throw"""
        lpoints = [params[PIdx.STR_PAN], params[PIdx.STR_LFT], params[PIdx.STR_ELB],
                   0, params[PIdx.STR_WRI], 1.58]

        # Current angle will be the ball pickup position
        l_cur_angle = self.PICKUP_POS + [1.58]
        r_cur_angle = l_cur_angle[:]
        r_cur_angle[0] *= -1
        r_cur_angle[5] = 3.14

        self.robot.interpolate('left', lpoints, current_angles=l_cur_angle)
        rpoints = lpoints[:]
        rpoints[0] *= -1
        rpoints[5] = 3.14
        self.robot.interpolate('right', rpoints, current_angles=r_cur_angle)

        collision = self.robot.check_collision()
        if collision:
            rospy.logwarn("Constraint Failed: DMP Collision")
            return False

        trajectory = []
        for idx in range(4):
            trajectory.append(self.generate_traj_points(idx, params))
        # Insert zeros in position for Wrist 1 and Wrist 3 joints
        length = len(trajectory[0])
        trajectory.insert(3, [0] * length)
        trajectory.append([0] * length)

        curr_angles = [lpoints, rpoints]
        self.robot.trajectory_learning(trajectory, current_angles=curr_angles)
        collision = self.robot.check_collision()
        if collision:
            rospy.logwarn("Constraint Failed: Throw Collision")
            return False

        return True

    def constraint(self, params):
        """Return (valid, fitness)"""
        if not(0 <= params[PIdx.D_ELBOW] < self.OVERALL_TIME):
            return (False, self.TIME_ERROR_REWARD)
        if not(0 <= params[PIdx.D_WRIST] < self.OVERALL_TIME):
            return (False, self.TIME_ERROR_REWARD)

        if not self.validate_time(params[PIdx.D_ELBOW], params[PIdx.STR_ELB], params[PIdx.END_ELB]):
            return (False, self.TIME_ERROR_REWARD)
        if not self.validate_time(params[PIdx.D_WRIST], params[PIdx.STR_WRI], params[PIdx.END_WRI]):
            return (False, self.TIME_ERROR_REWARD)

        if not self.validate_joint_limits([params[PIdx.STR_PAN]], self.PAN_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        if not self.validate_joint_limits([params[PIdx.STR_LFT], params[PIdx.END_LFT]], self.LIFT_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        if not self.validate_joint_limits([params[PIdx.STR_ELB], params[PIdx.END_ELB]], self.ELBOW_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        if not self.validate_joint_limits([params[PIdx.STR_WRI], params[PIdx.END_WRI]], self.WRIST_RESTRICT):
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
        self.robot.visualize_trajectory(False)
        ans = raw_input('Was the gazebo movement valid: ')
        if 'n' in ans:
            return tuple([r * 0.6 for r in self.INVALID_GAZEBO_REWARD])
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
        # Needs some delay before we get the reward
        time.sleep(4)
        self.track.stop()

        reward = self.track.get_reward()
        rospy.loginfo("Got reward: {}".format(reward))

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
        no_points = int(self.OVERALL_TIME / self.TIMESTEP)
        # Pan joint
        if joint_idx == 0:
            angle = params[PIdx.STR_PAN]
            return [angle] * no_points

        # Lift Joint
        if joint_idx == 1:
            angle = [params[PIdx.STR_LFT], params[PIdx.END_LFT]]
            start_points = 0
            dmp_points = no_points
            initial_points = int(self.LIFT_POINTS[1] - self.LIFT_POINTS[0])

        # Elbow Joint
        if joint_idx == 2:
            angle = [params[PIdx.STR_ELB], params[PIdx.END_ELB]]
            start_points = int(params[PIdx.D_ELBOW] / self.TIMESTEP)
            dmp_points = no_points - start_points
            initial_points = int(self.ELBOW_POINTS[1] - self.ELBOW_POINTS[0])

        # Wrist Joint
        if joint_idx == 3:
            angle = [params[PIdx.STR_WRI], params[PIdx.END_WRI]]
            start_points = int(params[PIdx.D_WRIST] / self.TIMESTEP)
            dmp_points = no_points - start_points
            initial_points = int(self.WRIST_POINTS[1] - self.WRIST_POINTS[0])

        points = []
        points += [angle[0]] * start_points

        weights = np.array([params[(PIdx.DMP_STR + (joint_idx - 1) * 15): (PIdx.DMP_STR + joint_idx * 15)]])
        tau = float(initial_points) / dmp_points

        self.dmps[joint_idx - 1].w = weights
        self.dmps[joint_idx - 1].y0 = np.array([angle[0]])
        self.dmps[joint_idx - 1].goal = np.array([angle[1]])

        result = self.dmps[joint_idx - 1].rollout(timesteps=dmp_points, tau=tau)[0]
        points += result.T.tolist()[0]

        return points

    def move_to_initial_position(self, params):
        if self.arm_state == ArmState.DMP:
            rospy.loginfo("-----------------------Initial Position-----------------------")
            points = [2.5, 0.75, -0.4, 0, 0, 1.58]
            self.robot.interpolate('left', points)
            points = [-2.5, 0.75, -0.4, 0, 0, 3.14]
            self.robot.interpolate('right', points)

            self.robot.visualize_trajectory(False)
            if not self.handle_input('Running move to initial position: '):
                return False

            self.robot.start_trajectory(delay=1)
            self.robot.wait_completion()

            rospy.loginfo("-----------------------Ball Pickup-----------------------")
            lpoints = self.PICKUP_POS + [1.58]
            self.robot.interpolate('left', lpoints, skip_joints=[JN.SHOULDER_LIFT])

            rpoints = lpoints[:]
            rpoints[0] *= -1
            rpoints[5] = 3.14
            self.robot.interpolate('right', rpoints, skip_joints=[JN.SHOULDER_LIFT])

            self.robot.visualize_trajectory(False)
            if not self.handle_input('Picking up the ball: '):
                return False

            self.robot.start_trajectory(delay=1)
            self.robot.wait_completion()

            self.robot.interpolate('left', lpoints)
            self.robot.interpolate('right', rpoints)

            self.robot.start_trajectory(delay=1)
            self.robot.wait_completion()

            self.arm_state = ArmState.INITIAL

        rospy.loginfo("-----------------------DMP Position-----------------------")
        lpoints = [params[PIdx.STR_PAN], params[PIdx.STR_LFT], params[PIdx.STR_ELB],
                   0, params[PIdx.STR_WRI], 1.58]

        rospy.loginfo('Generated DMP point: ')
        self.robot.print_numpy_array(lpoints)

        self.robot.interpolate('left', lpoints)

        rpoints = lpoints[:]
        rpoints[0] *= -1
        rpoints[5] = 3.14
        self.robot.interpolate('right', rpoints)

        self.robot.visualize_trajectory(False)
        if not self.handle_input('Running move to initial dmp position: '):
            return False

        collision = self.robot.check_collision()
        if collision:
            return False

        self.robot.start_trajectory(delay=1)
        self.robot.wait_completion()
        self.arm_state = ArmState.DMP
        return True

    def handle_input(self, out):
        print(out)
        exit = False
        while not exit:
            key = raw_input('Enter key: ')
            print('')
            if 'n' in key:
                return False
            elif '1' in key:
                print('Left: ', end="")
                self.robot.print_joint_trajectory(self.robot.left._goal.trajectory)
                print('Right: ', end="")
                self.robot.print_joint_trajectory(self.robot.right._goal.trajectory)
            elif '2' in key:
                print('Left: ', end="")
                self.robot.print_numpy_array(self.robot.left.get_current_joint_angles())
                print('Right: ', end="")
                self.robot.print_numpy_array(self.robot.right.get_current_joint_angles())
            else:
                exit = True
        return True


class EvaluateHansen(Evaluate):
    DMP_RESTRICT = [-1000, 1000]
    DMP_RANGE = [0, 40]
    MAX_VALID_REWARD = 600

    def __init__(self, *args, **kwargs):
        self.run_count = 0
        super(EvaluateHansen, self).__init__(*args, **kwargs)

    def scale(self, val, src, dst):
        """
        Scale the given value from the scale of src to the scale of dst.
        """
        return ((val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]

    def scale_params(self, params):
        in_range = [0, 10]
        scaled_params = [0] * len(params)
        scaled_params[0] = self.scale(params[0], in_range, [0, self.OVERALL_TIME])
        scaled_params[1] = self.scale(params[1], in_range, [0, self.OVERALL_TIME])
        scaled_params[2] = self.scale(params[2], in_range, self.PAN_RESTRICT)
        scaled_params[3] = self.scale(params[3], in_range, self.LIFT_RESTRICT)
        scaled_params[4] = self.scale(params[4], in_range, self.LIFT_RESTRICT)
        scaled_params[5] = self.scale(params[5], in_range, self.ELBOW_RESTRICT)
        scaled_params[6] = self.scale(params[6], in_range, self.ELBOW_RESTRICT)
        scaled_params[7] = self.scale(params[7], in_range, self.WRIST_RESTRICT)
        scaled_params[8] = self.scale(params[8], in_range, self.WRIST_RESTRICT)

        for i in range(9, len(params)):
            scaled_params[i] = self.scale(params[i], self.DMP_RANGE, self.DMP_RESTRICT)
        return scaled_params

    def get_initial_params(self):
        out_range = [0, 10]
        params = super(EvaluateHansen, self).get_initial_params()
        scaled_params = [0] * len(params)
        scaled_params[0] = self.scale(params[0], [0, self.OVERALL_TIME], out_range)
        scaled_params[1] = self.scale(params[1], [0, self.OVERALL_TIME], out_range)
        scaled_params[2] = self.scale(params[2], self.PAN_RESTRICT, out_range)
        scaled_params[3] = self.scale(params[3], self.LIFT_RESTRICT, out_range)
        scaled_params[4] = self.scale(params[4], self.LIFT_RESTRICT, out_range)
        scaled_params[5] = self.scale(params[5], self.ELBOW_RESTRICT, out_range)
        scaled_params[6] = self.scale(params[6], self.ELBOW_RESTRICT, out_range)
        scaled_params[7] = self.scale(params[7], self.WRIST_RESTRICT, out_range)
        scaled_params[8] = self.scale(params[8], self.WRIST_RESTRICT, out_range)

        for i in range(9, len(params)):
            scaled_params[i] = self.scale(params[i], self.DMP_RESTRICT, self.DMP_RANGE)
        return scaled_params

    def check_feasible(self, params, f):
        if f is not None:
            if any([v > 1000 for v in f]):
                rospy.logwarn('Failed fitness value {}'.format(f))
                return False
            else:
                rospy.logwarn('Skip feasibility, good reward')
                return True

        scaled_params = self.scale_params(params)
        (valid, fitness) = self.constraint(scaled_params)
        if not valid:
            rospy.logwarn('Constraint Failed: Invalid Params')
            return valid

        lpoints = [scaled_params[PIdx.STR_PAN], scaled_params[PIdx.STR_LFT],
                   scaled_params[PIdx.STR_ELB], 0, scaled_params[PIdx.STR_WRI], 1.58]
        rpoints = lpoints[:]
        rpoints[0] *= -1
        rpoints[5] = 3.14

        dist = self.robot.get_dist_btw_effectors(lpoints, rpoints)
        if not (self.BALL_GRASP_DISTANCE[0] <= dist <= self.BALL_GRASP_DISTANCE[1]):
            rospy.logwarn('Constraint Failed: Eff distance: {}'.format(dist))
            return False

        rospy.loginfo('Eff distance: {}'.format(dist))

        # Check collision
        if not self.collision_constraint(scaled_params):
            return False
        return True

    def eval(self, params):
        # return [np.random.random() * 1000]
        scaled_params = self.scale_params(params)
        if not self.check_feasible(params, None):
            return [10000]

        fitness = super(EvaluateHansen, self).eval(scaled_params)
        # Hansen only minimizes, so convert all negative values to positive values
        val = fitness[0]
        if val < 0:
            # Means a invalid function evaluation
            reward = [-val]
        else:
            # A valid run
            reward = [self.MAX_VALID_REWARD - val]
        rospy.loginfo("Reward: {}".format(reward))
        return reward
