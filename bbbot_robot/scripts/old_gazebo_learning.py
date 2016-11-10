#! /usr/bin/env python
from bbbot_robot.cmaes import setup_cmaes, run_cmaes, checkpoint_handle, generate_name, init_creator
from bbbot_robot.cmaes_hansen import hans_setup_cmaes, hans_run_cmaes
from bbbot_robot.evaluate import Evaluate, EvaluateHansen, EvaluateGazebo, PIdx
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN
import sys
import os
import rospy
from sympy import Point3D, Line3D, N
from gazebo_msgs.srv import SetModelStateRequest, SetModelState


cmaes_dump = "/home/nikhilkalige/workspace/asu/thesis/cmaes_dump"
NGEN = 1000
SIGMA = .7
DMP = 15
BAG_FILE = '/home/nikhilkalige/workspace/ros/devel_packages/bag_files/2016-05-15-15-20-40.bag'


class GazeboRun(Evaluate):
    # SPANW POINT -x 0.235553304733924 -y 0 -z 1.50165354067886
    def __init__(self):
        self.robot = Robot(use_prefix=True, display=False, sim=True, collision=False)
        self.init_dmp(DMP, BAG_FILE)

    def run(self, params):
        points = [2.5, 0.75, -0.4, 0, 0, 1.58]
        self.robot.interpolate('left', points, delay=0.05)
        points = [-2.5, 0.75, -0.4, 0, 0, 3.14]
        self.robot.interpolate('right', points, delay=0.05)

        # self.get_spawn_position(points, points)
        self.robot.start_trajectory(delay=0.1)
        self.robot.wait_completion()

        lpoints = self.PICKUP_POS + [1.58]
        self.robot.interpolate('left', lpoints, skip_joints=[JN.SHOULDER_LIFT], delay=0.05)

        rpoints = lpoints[:]
        rpoints[0] *= -1
        rpoints[5] = 3.14
        self.robot.interpolate('right', rpoints, skip_joints=[JN.SHOULDER_LIFT], delay=0.05)

        self.robot.start_trajectory(delay=0.1)
        self.robot.wait_completion()

        self.robot.interpolate('left', lpoints)
        self.robot.interpolate('right', rpoints)

        # self.get_spawn_position(lpoints, rpoints)
        self.robot.start_trajectory(delay=0.1)
        self.robot.wait_completion()

        lpoints = [params[PIdx.STR_PAN], params[PIdx.STR_LFT], params[PIdx.STR_ELB],
                   0, params[PIdx.STR_WRI], 1.58]

        self.robot.interpolate('left', lpoints)

        rpoints = lpoints[:]
        rpoints[0] *= -1
        rpoints[5] = 3.14
        self.robot.interpolate('right', rpoints)

        # self.get_spawn_position(lpoints, rpoints)
        self.robot.start_trajectory(delay=0.1)
        self.robot.wait_completion()

        self.spawn_ball(lpoints, rpoints)
        return True

    def throw(self, params):
        trajectory = []
        for idx in range(4):
            trajectory.append(self.generate_traj_points(idx, params))
        # Insert zeros in position for Wrist 1 and Wrist 3 joints
        length = len(trajectory[0])
        trajectory.insert(3, [0] * length)
        trajectory.append([0] * length)
        self.robot.trajectory_learning(trajectory)
        self.robot.start_trajectory(2)
        self.robot.wait_completion()

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
            s = N(intersect_pt[0])
            print "-x {} -y {} -z {}".format(s[0], s[1], s[2])
            return s
        else:
            print "Nothing found"

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


tbest = [1.8284905472974833, 3.8234043731516563, 4.4687962002694936, 5.9093902481269245, 6.3621251606771105, 7.1364029283308801, 3.7337401286852314, 5.8413234871325308, 3.5789461685557429, 17.688036571316168, 16.531423892854548, 15.931914016162819, 19.690284533171553, 15.524394584369295, 19.381155271817349, 18.509201347466515, 17.555922115366588, 18.959110688528355, 20.531785886779485, 18.389557344497362, 23.003878789659257, 18.145402218726701, 20.743144623815809, 21.402787391827779, 18.130823505093915, 17.401271236647492, 17.324411851191957, 16.950671791381737, 15.724880455457042, 16.89560499566506, 16.038575485729545, 14.860801025435059, 18.048539319969887, 15.983098143750244, 19.637839665936362, 19.895032080005606, 19.571717513041257, 22.128638479122625, 21.36623151735505, 16.960825149789677, 16.461194485926239, 17.313116268705937, 15.438493399683432, 15.389238189216485, 14.50499980753891, 15.903906490062029, 16.451954566493306, 18.473493479123363, 17.080038548510114, 18.886396942571647, 20.884414857503216, 20.343970031654514, 19.622613628033911, 19.193384997149437]


def cmaes_hansen(robot, location):
    cmaes = hans_setup_cmaes(SIGMA, robot.get_initial_params(), constraint_func=True)
    # p = robot.get_initial_params()
    # robot.eval(tbest)
    # return
    # filename = '/home/lonewolf/workspace/asu/thesis/cmaes_dump/12_06_15_19_21/cma_00021.pkl'
    #filename = '/home/nikhilkalige/workspace/asu/thesis/cmaes_dump/19_07_16_09_17/cma_00020.pkl'
    #cmaes = hans_setup_cmaes(SIGMA, robot.get_initial_params(), constraint_func=True, checkpoint=True, filename=filename)

    hans_run_cmaes(cmaes, robot.eval, location)


if __name__ == '__main__':
    rospy.init_node('bot', log_level=rospy.WARN)
    robot = EvaluateGazebo(DMP, BAG_FILE, plot=True)
    rospy.loginfo("Finished initialization")

    location = generate_name(cmaes_dump)
    rospy.loginfo("Location: {}".format(location))

    cmaes_hansen(robot, location)

    robot.track.kill()
    rospy.loginfo("Finished execution of cmaes")


# if __name__ == '__main__':
#     rospy.init_node('bot')
#     g = GazeboRun()
#     # p = g.get_initial_params()
#     g.run(EvaluateHansen.scale_params(tbest))
#     g.throw(EvaluateHansen.scale_params(tbest))
#     # g.run(p)
#     # g.throw(p)
