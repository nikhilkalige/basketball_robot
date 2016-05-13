from .arm import Arm
from bbbot_collision.srv import CollisionCheckRequest, CollisionCheckResponse, CollisionCheck
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy
import numpy as np
import sys
from enum import IntEnum


class JointNames(IntEnum):
    SHOULDER_PAN = 0
    SHOULDER_LIFT = 1
    ELBOW = 2
    WRIST_1 = 3
    WRIST_2 = 4
    WRIST_3 = 5


def find_nearest(array, value):
    idx = (np.abs(array - value)).argmin()
    return idx


class Robot:
    def __init__(self, use_prefix=False, sim=False, single_arm=False, pos_controller=True):
        self.sim = sim
        self.collsion_service = False
        self.single_arm = single_arm

        self.left = Arm('LeftArm', 'leftarm', use_prefix=use_prefix, sim=sim, pos_controller=pos_controller)
        if not self.single_arm:
            self.right = Arm('RightArm', 'rightarm', use_prefix=use_prefix, sim=sim, pos_controller=pos_controller)

    def control_torque(self, enable=True):
        if not self.left.control_torque(enable):
            return False
        if not self.single_arm:
            if not self.right.control_torque(enable):
                return False
        return True

    def create_trajectory(self, arm_str, index, end_point, increment, time=1):
        arm = getattr(self, arm_str)
        arm.init_trajectory()
        current_angles = arm.get_current_joint_angles()

        if increment == 0.0:
            return

        arm.logmsg("Generated angle values")
        pts = self.generate_points(current_angles, index, end_point, increment)

        for pt in pts:
            print ["{:.7f}".format(i) for i in pt]

        if not self.sim:
            validate = raw_input("Validate generated angle values: \n")
            if 'n' in validate.lower():
                sys.exit(1)

        for pt in pts:
            arm.add_traj_point(pt, time)

    def generate_points(self, current_angles, index, end_point, increment):
        start_point = current_angles[index]
        points = []
        increment = abs(increment)

        no_of_points = float(np.abs(start_point - end_point)) / np.abs(increment)
        no_of_points = np.ceil(no_of_points)
        if no_of_points == 1:
            no_of_points = 2

        for pt in np.linspace(start_point, end_point, no_of_points):
            current_angles[index] = pt
            points.append(current_angles[:])

        return points

    def modify_trajectory(self, arm_str, points):
        """ The points should be of the format
            [
                {arm_index: start_degree, ......},
                {arm_index: end_degree, ......}
            ]
        """
        arm = getattr(self, arm_str)
        # Find the start and end point of shoulder pan joint
        main_start_pt = points[0][JointNames.SHOULDER_PAN]
        main_end_pt = points[-1][JointNames.SHOULDER_PAN]

        # Points for SHOULDER_PAN
        pan_positions = []
        for pt in arm._goal.trajectory.points:
            pan_positions.append(pt.positions[JointNames.SHOULDER_PAN])

        # Find the closest index
        main_start_idx = find_nearest(pan_positions, np.deg2rad(main_start_pt))
        main_end_idx = find_nearest(pan_positions, np.deg2rad(main_end_pt))

        # no_of_points = (main_end_idx - main_start_idx) + 1
        # Remove the PAN joint
        for pt in points:
            pt.pop(JointNames.SHOULDER_PAN)

        for key in points[0]:
            start_pt = np.deg2rad(points[0][key])
            end_pt = np.deg2rad(points[-1][key])

            no_of_points = float(np.abs(start_pt - end_pt)) / np.abs(increment)
            no_of_points = np.ceil(no_of_points)
            if no_of_points == 1:
                no_of_points = 2

            # pos_idx = main_start_idx
            pos_idx = main_end_idx
            for pt in np.linspace(end_pt, start_pt, no_of_points):
                arm._goal.trajectory.points[pos_idx].positions[key] = pt
                pos_idx = pos_idx - 1
                if pos_idx == -1:
                    break

            # Set the last n values to the last value
            pos_idx = main_end_idx
            last_value = arm._goal.trajectory.points[pos_idx].positions[key]
            for pos in arm._goal.trajectory.points[pos_idx:]:
                pos.positions[key] = last_value

        for pt in arm._goal.trajectory.points:
            print ["{:.7f}".format(i) for i in pt.positions]

        if not self.sim:
            validate = raw_input("Validate generated angle values: \n")
            if 'n' in validate.lower():
                sys.exit(1)

    def start_trajectory(self, delay=3):
        start_time = rospy.Time.now() + rospy.Duration(delay)
        self.left.send_trajectory(start_time)
        if not self.single_arm:
            self.right.send_trajectory(start_time)

    def wait_completion(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.left.trajectory_active:
                if self.single_arm:
                    return
                if not self.single_arm and not self.right.trajectory_active:
                    return
            rate.sleep()

    def check_collision(self):
        # Create a new trajectory by combining both the trajectories, assumption is that the timing is same
        # The order of joining should be [left + right]
        traj = JointTrajectory()
        traj.joint_names = self.left._goal.trajectory.joint_names + self.right._goal.trajectory.joint_names

        no_points = len(self.left._goal.trajectory.points)
        for index in xrange(no_points):
            pt = JointTrajectoryPoint()
            left_pt = self.left._goal.trajectory.points[index]
            right_pt = self.right._goal.trajectory.points[index]

            pt.positions = left_pt.positions + right_pt.positions
            pt.velocities = left_pt.velocities + right_pt.velocities
            pt.time_from_start = left_pt.time_from_start
            traj.points.append(pt)

        msg = CollisionCheckRequest()
        msg.trajectory = traj

        try:
            collision_service = rospy.ServiceProxy("collision_check", CollisionCheck)
            resp = collision_service(msg)
            if resp.collision_found:
                rospy.logwarn("Collision Found")
            else:
                rospy.loginfo("Collision not found, good to go :)")
            return resp.collision_found
        except rospy.ServiceException as e:
            rospy.logwarn("Exception on collision check {}".format(e))
            return True

    def adjust_traj_length(self):
        if len(self.left._goal.trajectory.points) == len(self.right._goal.trajectory.points):
            return
        if len(self.left._goal.trajectory.points) < len(self.right._goal.trajectory.points):
            lower = self.left._goal.trajectory.points
            upper = self.right._goal.trajectory.points
        else:
            lower = self.right._goal.trajectory.points
            upper = self.left._goal.trajectory.points

        count = len(upper) - len(lower)
        lower_count = len(lower)
        last_point = lower[lower_count - 1]

        for i in xrange(count):
            pt = JointTrajectoryPoint()
            pt.positions = last_point.positions[:]
            pt.velocities = last_point.velocities[:]
            pt.time_from_start = upper[lower_count + i].time_from_start

            lower.append(pt)


