from __future__ import print_function
from bbbot_robot.evaluate import EvaluateHansen, EvaluateGazebo
import numpy as np
import rospy
from enum import IntEnum
import zmq
import rosbag
from bbbot_robot.robot import Robot
from bbbot_robot.config_reader import conf
import sys
from bbbot_robot.robot import JointNames as JN
from bbbot_robot.tracking import Tracker
from bbbot_robot.ball_tracker import BallTracker, BasketTracker
from pydmps.dmp_discrete import DMPs_discrete
from sympy import Point3D, Line3D, N
from gazebo_msgs.srv import SetModelStateRequest, SetModelState

'''
Parameters:
1. Start and end angle for pan joint
2. Start and end angle for lift joint
3. Start and end angle for elbow joint
4. Start and end angle for wrist joint
5. Weights of dmps learned from the bag file. Decided on 15 per joint angle

The above params are repeated for left and right arm.

Order of params
[startPan, endPan, startLift, endLift, startElbow, endElbow, startWrist, endWrist,
weightsPan, weightsLift, weightsElbow, weightsWrist] * 2

Totally = (8 + 4 * 15) * 2 = 68 * 2 = 136
'''


class PIdx(IntEnum):
    L_STR_PAN = 0
    L_END_PAN = 1
    L_STR_LFT = 2
    L_END_LFT = 3
    L_STR_ELB = 4
    L_END_ELB = 5
    L_STR_WRI = 6
    L_END_WRI = 7
    L_DMP_STR = 8
    L_DMP_END = 67
    R_STR_PAN = 68
    R_END_PAN = 69
    R_STR_LFT = 70
    R_END_LFT = 71
    R_STR_ELB = 72
    R_END_ELB = 73
    R_STR_WRI = 74
    R_END_WRI = 75
    R_DMP_STR = 76
    R_DMP_END = 135


class EvaluateDual(EvaluateHansen):
    DEFAULT_L_PARAMS = [1.697875, 1.697875, 0.2946925, 0.9123826, 1.660619, 0.3703937, 0.5213307, -0.529045]
    DEFAULT_R_PARAMS = [-1.697875, -1.697875, 0.2946925, 0.9123826, 1.660619, 0.3703937, 0.5213307, -0.529045]

    def __init__(self, *args, **kwargs):
        super(EvaluateDual, self).__init__(*args, **kwargs)

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
        valid_points.append(np.array([self.DEFAULT_L_PARAMS[0]] * 60))
        length.append(60)

        points_idx = [self.LIFT_POINTS, self.ELBOW_POINTS, self.WRIST_POINTS]
        for idx, arr in zip(points_idx, position):
            valid_points.append(arr[idx[0]:idx[1]])
            length.append(idx[1] - idx[0])

        valid_points.append(np.array([self.DEFAULT_R_PARAMS[0]] * 60))
        length.append(60)

        for i in [1, 2, 3]:
            valid_points.append(valid_points[i])
            length.append(length[i])

        self.dmps = []
        for l, joint in zip(length, valid_points):
            dmp = DMPs_discrete(dmps=1, bfs=dmp_count, dt=1.0 / l)
            dmp.imitate_path(joint)
            self.dmps.append(dmp)

    def get_initial_params(self):
        weights = []
        for dmp in self.dmps:
            weights += dmp.w[0].tolist()

        w_len = len(weights)
        params =  self.DEFAULT_L_PARAMS + weights[0:w_len/2] + self.DEFAULT_R_PARAMS + weights[w_len/2:]
        return self.rescale_params(params)

    @classmethod
    def scale_params(cls, params):
        """Converts to range needed for execution"""
        in_range = [0, 10]
        scaled_params = [0] * len(params)

        scaled_params[0] = cls.scale(params[0], in_range, cls.PAN_RESTRICT)
        scaled_params[1] = cls.scale(params[1], in_range, cls.PAN_RESTRICT)
        scaled_params[2] = cls.scale(params[2], in_range, cls.LIFT_RESTRICT)
        scaled_params[3] = cls.scale(params[3], in_range, cls.LIFT_RESTRICT)
        scaled_params[4] = cls.scale(params[4], in_range, cls.ELBOW_RESTRICT)
        scaled_params[5] = cls.scale(params[5], in_range, cls.ELBOW_RESTRICT)
        scaled_params[6] = cls.scale(params[6], in_range, cls.WRIST_RESTRICT)
        scaled_params[7] = cls.scale(params[7], in_range, cls.WRIST_RESTRICT)

        neg_pan_restrict = [-1 * x for x in cls.PAN_RESTRICT]
        scaled_params[68] = cls.scale(params[68], in_range, neg_pan_restrict)
        scaled_params[69] = cls.scale(params[69], in_range, neg_pan_restrict)
        scaled_params[70] = cls.scale(params[70], in_range, cls.LIFT_RESTRICT)
        scaled_params[71] = cls.scale(params[71], in_range, cls.LIFT_RESTRICT)
        scaled_params[72] = cls.scale(params[72], in_range, cls.ELBOW_RESTRICT)
        scaled_params[73] = cls.scale(params[73], in_range, cls.ELBOW_RESTRICT)
        scaled_params[74] = cls.scale(params[74], in_range, cls.WRIST_RESTRICT)
        scaled_params[75] = cls.scale(params[75], in_range, cls.WRIST_RESTRICT)

        for i in range(8, 68):
            scaled_params[i] = cls.scale(params[i], cls.DMP_RANGE, cls.DMP_RESTRICT)

        for i in range(76, 136):
            scaled_params[i] = cls.scale(params[i], cls.DMP_RANGE, cls.DMP_RESTRICT)

        return scaled_params

    @classmethod
    def rescale_params(cls, params):
        """Converts towards cmaes range"""
        out_range = [0, 10]
        scaled_params = [0] * len(params)

        scaled_params[0] = cls.scale(params[0], cls.PAN_RESTRICT, out_range)
        scaled_params[1] = cls.scale(params[1], cls.PAN_RESTRICT, out_range)
        scaled_params[2] = cls.scale(params[2], cls.LIFT_RESTRICT, out_range)
        scaled_params[3] = cls.scale(params[3], cls.LIFT_RESTRICT, out_range)
        scaled_params[4] = cls.scale(params[4], cls.ELBOW_RESTRICT, out_range)
        scaled_params[5] = cls.scale(params[5], cls.ELBOW_RESTRICT, out_range)
        scaled_params[6] = cls.scale(params[6], cls.WRIST_RESTRICT, out_range)
        scaled_params[7] = cls.scale(params[7], cls.WRIST_RESTRICT, out_range)

        neg_pan_restrict = [-1 * x for x in cls.PAN_RESTRICT]
        scaled_params[68] = cls.scale(params[68], neg_pan_restrict, out_range)
        scaled_params[69] = cls.scale(params[69], neg_pan_restrict, out_range)
        scaled_params[70] = cls.scale(params[70], cls.LIFT_RESTRICT, out_range)
        scaled_params[71] = cls.scale(params[71], cls.LIFT_RESTRICT, out_range)
        scaled_params[72] = cls.scale(params[72], cls.ELBOW_RESTRICT, out_range)
        scaled_params[73] = cls.scale(params[73], cls.ELBOW_RESTRICT, out_range)
        scaled_params[74] = cls.scale(params[74], cls.WRIST_RESTRICT, out_range)
        scaled_params[75] = cls.scale(params[75], cls.WRIST_RESTRICT, out_range)

        for i in range(8, 68):
            scaled_params[i] = cls.scale(params[i], cls.DMP_RESTRICT, cls.DMP_RANGE)

        for i in range(76, 136):
            scaled_params[i] = cls.scale(params[i], cls.DMP_RESTRICT, cls.DMP_RANGE)
        return scaled_params

    def constraint(self, params):
        """Return (valid, fitness)"""
        if not self.validate_joint_limits([params[PIdx.L_STR_PAN], params[PIdx.L_END_PAN]], self.PAN_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        if not self.validate_joint_limits([params[PIdx.L_STR_LFT], params[PIdx.L_END_LFT]], self.LIFT_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        if not self.validate_joint_limits([params[PIdx.L_STR_ELB], params[PIdx.L_END_ELB]], self.ELBOW_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        if not self.validate_joint_limits([params[PIdx.L_STR_WRI], params[PIdx.L_END_WRI]], self.WRIST_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)

        right_pan_restrict = tuple(-1 * x for x in self.PAN_RESTRICT)
        if not self.validate_joint_limits([params[PIdx.R_STR_PAN], params[PIdx.R_END_PAN]], right_pan_restrict):
            return (False, self.JOINT_ERROR_REWARD)
        if not self.validate_joint_limits([params[PIdx.R_STR_LFT], params[PIdx.R_END_LFT]], self.LIFT_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        if not self.validate_joint_limits([params[PIdx.R_STR_ELB], params[PIdx.R_END_ELB]], self.ELBOW_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        if not self.validate_joint_limits([params[PIdx.R_STR_WRI], params[PIdx.R_END_WRI]], self.WRIST_RESTRICT):
            return (False, self.JOINT_ERROR_REWARD)
        return (True, tuple())

    def generate_traj_points(self, joint_idx, params, leftarm=True):
        no_points = int(self.OVERALL_TIME / self.TIMESTEP)
        # Pan joint
        if joint_idx == 0:
            if leftarm:
                angle = [params[PIdx.L_STR_PAN], params[PIdx.L_END_PAN]]
            else:
                angle = [params[PIdx.R_STR_PAN], params[PIdx.R_END_PAN]]
            start_points = 0
            dmp_points = no_points
            initial_points = no_points

        # Lift Joint
        if joint_idx == 1:
            if leftarm:
                angle = [params[PIdx.L_STR_LFT], params[PIdx.L_END_LFT]]
            else:
                angle = [params[PIdx.R_STR_LFT], params[PIdx.R_END_LFT]]
            start_points = 0
            dmp_points = no_points
            initial_points = int(self.LIFT_POINTS[1] - self.LIFT_POINTS[0])

        # Elbow Joint
        if joint_idx == 2:
            if leftarm:
                angle = [params[PIdx.L_STR_ELB], params[PIdx.L_END_ELB]]
            else:
                angle = [params[PIdx.R_STR_ELB], params[PIdx.R_END_ELB]]
            start_points = 0
            dmp_points = no_points - start_points
            initial_points = int(self.ELBOW_POINTS[1] - self.ELBOW_POINTS[0])

        # Wrist Joint
        if joint_idx == 3:
            if leftarm:
                angle = [params[PIdx.L_STR_WRI], params[PIdx.L_END_WRI]]
            else:
                angle = [params[PIdx.R_STR_WRI], params[PIdx.R_END_WRI]]
            start_points = 0
            dmp_points = no_points - start_points
            initial_points = int(self.WRIST_POINTS[1] - self.WRIST_POINTS[0])

        points = []
        points += [angle[0]] * start_points

        if leftarm:
            weights = np.array([params[(PIdx.L_DMP_STR + joint_idx * 15): (PIdx.L_DMP_STR + (joint_idx + 1) * 15)]])
        else:
            weights = np.array([params[(PIdx.R_DMP_STR + joint_idx * 15): (PIdx.R_DMP_STR + (joint_idx + 1) * 15)]])

        tau = float(initial_points) / dmp_points

        self.dmps[joint_idx - 1].w = weights
        self.dmps[joint_idx - 1].y0 = np.array([angle[0]])
        self.dmps[joint_idx - 1].goal = np.array([angle[1]])

        result = self.dmps[joint_idx - 1].rollout(timesteps=dmp_points, tau=tau)[0]
        points += result.T.tolist()[0]

        return points

    def collision_constraint(self, params):
        """ Will check for two collsions, ball pick to dmp and dmp to throw"""
        lpoints = [params[PIdx.L_STR_PAN], params[PIdx.L_STR_LFT], params[PIdx.L_STR_ELB],
                   0, params[PIdx.L_STR_WRI], 1.58]
        rpoints = [params[PIdx.R_STR_PAN], params[PIdx.R_STR_LFT], params[PIdx.R_STR_ELB],
                   0, params[PIdx.R_STR_WRI], 3.14]

        # Current angle will be the ball pickup position
        l_cur_angle = self.PICKUP_POS + [1.58]
        r_cur_angle = l_cur_angle[:]
        r_cur_angle[0] *= -1
        r_cur_angle[5] = 3.14

        print(lpoints, rpoints)

        self.robot.interpolate('left', lpoints, current_angles=l_cur_angle)
        self.robot.interpolate('right', rpoints, current_angles=r_cur_angle)

        collision = self.robot.check_collision()
        if collision:
            rospy.logwarn("Constraint Failed: DMP Collision")
            return False

        left_trajectory, right_trajectory = [], []
        for idx in range(4):
            left_trajectory.append(self.generate_traj_points(idx, params))
            right_trajectory.append(self.generate_traj_points(idx, params, False))

        # Insert zeros in position for Wrist 1 and Wrist 3 joints
        length = len(left_trajectory[0])
        left_trajectory.insert(3, [0] * length)
        left_trajectory.append([0] * length)

        right_trajectory.insert(3, [0] * length)
        right_trajectory.append([0] * length)

        curr_angles = [lpoints, rpoints]
        self.robot.dual_trajectory_learning(left_trajectory, right_trajectory, current_angles=curr_angles)
        collision = self.robot.check_collision()
        if collision:
            rospy.logwarn("Constraint Failed: Throw Collision")
            return False

        return True

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

        lpoints = [scaled_params[PIdx.L_STR_PAN], scaled_params[PIdx.L_STR_LFT],
                   scaled_params[PIdx.L_STR_ELB], 0, scaled_params[PIdx.L_STR_WRI], 1.58]
        rpoints = [scaled_params[PIdx.R_STR_PAN], scaled_params[PIdx.R_STR_LFT],
                   scaled_params[PIdx.R_STR_ELB], 0, scaled_params[PIdx.R_STR_WRI], 3.14]

        dist = self.robot.get_dist_btw_effectors(lpoints, rpoints)
        if not (self.BALL_GRASP_DISTANCE[0] <= dist <= self.BALL_GRASP_DISTANCE[1]):
            rospy.logwarn('Constraint Failed: Eff distance: {}'.format(dist))
            return False

        rospy.loginfo('Eff distance: {}'.format(dist))

        # Check collision
        if not self.collision_constraint(scaled_params):
            return False
        return True

    def eval(self, params, mean_values=False):
        """Params comes from the cmaes evaluate"""
        scaled_params = self.scale_params(params)
        if not self.check_feasible(params, None):
            return [10000]

        (valid, fitness) = self.constraint(scaled_params)
        if not valid:
            rospy.logwarn("Constraint failed")
            return fitness

        # Pick the ball
        # Move to the initial position
        l_str_point = [scaled_params[PIdx.L_STR_PAN], scaled_params[PIdx.L_STR_LFT],
                       scaled_params[PIdx.L_STR_ELB], 0, scaled_params[PIdx.L_STR_WRI], 1.58]
        r_str_point = [scaled_params[PIdx.R_STR_PAN], scaled_params[PIdx.R_STR_LFT],
                       scaled_params[PIdx.R_STR_ELB], 0, scaled_params[PIdx.R_STR_WRI], 3.14]
        if not self.move_to_initial_position([], np.array(l_str_point), np.array(r_str_point)):
            return tuple([r * 0.8 for r in self.COLLISION_REWARD])

        # Start the throwing process
        left_trajectory, right_trajectory = [], []
        for idx in range(4):
            left_trajectory.append(self.generate_traj_points(idx, scaled_params))
            right_trajectory.append(self.generate_traj_points(idx, scaled_params, False))

        # Insert zeros in position for Wrist 1 and Wrist 3 joints
        length = len(left_trajectory[0])
        left_trajectory.insert(3, [0] * length)
        left_trajectory.append([0] * length)

        right_trajectory.insert(3, [0] * length)
        right_trajectory.append([0] * length)

        self.robot.dual_trajectory_learning(left_trajectory, right_trajectory)

        fitness = self.visualize()
        if not fitness:
            fitness = self.real_run()

        val = fitness[0]
        if val < 0:
            # Means a invalid function evaluation
            reward = [-val]
        else:
            # A valid run
            reward = [self.MAX_VALID_REWARD - val]
        rospy.loginfo("Reward: {}".format(reward))

        # Compute original params list that matches the type of CMAES evaluation, helps
        # in sending the message
        orig_params = [0, 0] + np.random.uniform(0, 3.14, 7).tolist() + [0] * 45
        self.send_msg(orig_params, np.random.uniform(0, 3.14, (3, 60)), reward, mean_values)

        if not mean_values:
            self.countevals += 1

        return reward

    def send_msg(self, params, dmp, fitness, mean_run=False):
        """ Params = list, fitness = float """
        params = np.array(params)
        md = dict(
            params_dtype=str(params.dtype),
            params_shape=params.shape,
            fitness=fitness,
            evaluations=self.countevals,
            dmp_dtype=str(dmp.dtype),
            dmp_shape=dmp.shape,
            mean_run=mean_run
        )
        try:
            self.socket.send_json(md, zmq.SNDMORE)
            self.socket.send(params, zmq.SNDMORE)
            self.socket.send(dmp)
        except:
            pass


class EvaluateDualGazebo(EvaluateDual):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault("gazebo", True)
        super(EvaluateDualGazebo, self).__init__(*args, **kwargs)

    @staticmethod
    def pose_to_line(pose1, pose2):
        p1 = Point3D(pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z)
        p2 = Point3D(pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z)

        return Line3D(p1, p2)

    def get_spawn_position(self, left_angles, right_angles):
        links = ['leftarm_wrist_3_link', 'leftarm_finger_link', 'rightarm_wrist_3_link', 'rightarm_finger_link']
        resp = self.robot.get_fk_service(left_angles, right_angles, links)
        if resp < 0:
            print("Error", resp)
            return -1

        l1 = self.pose_to_line(resp.pose_stamped[0], resp.pose_stamped[1])
        l2 = self.pose_to_line(resp.pose_stamped[2], resp.pose_stamped[3])
        intersect_pt = l1.intersection(l2)
        if intersect_pt:
            return N(intersect_pt[0])
            # print "-x {} -y {} -z {}".format(s[0], s[1], s[2])
        else:
            rospy.logwarn("Can't spawn ball")
            return None

    def spawn_ball(self, left_angles, right_angles):
        pos = self.get_spawn_position(left_angles, right_angles)
        if not pos:
            return False

        msg = SetModelStateRequest()
        msg.model_state.model_name = "basketball"
        msg.model_state.pose.position.x = pos[0]
        msg.model_state.pose.position.y = pos[1]
        msg.model_state.pose.position.z = pos[2]

        while True:
            try:
                model_service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
                resp = model_service(msg)
                if resp.success:
                    break
            except rospy.ServiceException as e:
                rospy.logwarn("Exception on set model state service {}".format(e))

        return True
