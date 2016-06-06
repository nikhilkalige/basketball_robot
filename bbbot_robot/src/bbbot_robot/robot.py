from .arm import Arm
from bbbot_collision.srv import CollisionCheckRequest, CollisionCheckResponse, CollisionCheck
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
import rospy
import numpy as np
import sys
from enum import IntEnum
import rosbag
import moveit_commander


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
    FIXED_POSITIONS = ['leftarm_shoulder_pan_joint', 'leftarm_shoulder_lift_joint', 'leftarm_elbow_joint',
                       'leftarm_wrist_1_joint', 'leftarm_wrist_2_joint', 'leftarm_wrist_3_joint',
                       'rightarm_shoulder_pan_joint', 'rightarm_shoulder_lift_joint', 'rightarm_elbow_joint',
                       'rightarm_wrist_1_joint', 'rightarm_wrist_2_joint', 'rightarm_wrist_3_joint'
                       ]

    def __init__(self, use_prefix=False, sim=False, single_arm=False, pos_controller=True, display=False):
        self.sim = sim
        self.collsion_service = False
        self.single_arm = single_arm
        # Show trajectory on rviz
        self.display = display

        self.left = Arm('LeftArm', 'leftarm', use_prefix=use_prefix, sim=sim, pos_controller=pos_controller)
        if not self.single_arm:
            self.right = Arm('RightArm', 'rightarm', use_prefix=use_prefix, sim=sim, pos_controller=pos_controller)

        if self.display:
            moveit_commander.roscpp_initialize(sys.argv)
            self.robot_model = moveit_commander.RobotCommander()
            self.rviz_pub = rospy.Publisher('/move_group/display_planned_path',
                                            DisplayTrajectory, queue_size=10)

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

    def modify_trajectory(self, arm_str, reference_joint, points, increment=0.06):
        """ The points should be of the format
            [
                {arm_index: start_degree, ......},
                {arm_index: end_degree, ......}
            ]

            If start_degree is not specified for the reference joint, then the joint speed will be maximized,
            else we will use the distance between the start and end points of the reference joint as an reference to
            calculate the no of points to be used, therefore the increment can be very large or very small.
        """
        arm = getattr(self, arm_str)
        # Find the start and end point of reference_joint
        main_start_pt = False
        if points[0].get(reference_joint):
            main_start_pt = points[0][reference_joint]
        main_end_pt = points[-1][reference_joint]

        # Positions for the reference_joint
        pan_positions = []
        for pt in arm._goal.trajectory.points:
            pan_positions.append(pt.positions[reference_joint])

        # Find the closest index
        main_end_idx = find_nearest(pan_positions, np.deg2rad(main_end_pt))
        if main_start_pt:
            main_start_idx = find_nearest(pan_positions, np.deg2rad(main_start_pt))
            no_of_points = (main_end_idx - main_start_idx) + 1

        # Remove the PAN joint
        for pt in points:
            if reference_joint in pt:
                pt.pop(reference_joint)

        for key in points[0]:
            start_pt = np.deg2rad(points[0][key])
            end_pt = np.deg2rad(points[-1][key])

            if main_start_pt:
                pos_idx = main_start_idx
            else:
                no_of_points = float(np.abs(start_pt - end_pt)) / np.abs(increment)
                no_of_points = np.ceil(no_of_points)
                if no_of_points == 1:
                    no_of_points = 2
                pos_idx = main_end_idx
                # reverse the startpt and endpt
                start_pt, end_pt = end_pt, start_pt

            for pt in np.linspace(start_pt, end_pt, no_of_points):
                arm._goal.trajectory.points[pos_idx].positions[key] = pt
                if main_start_pt:
                    pos_idx = pos_idx + 1
                else:
                    pos_idx = pos_idx - 1
                    if pos_idx == -1:
                        break

            # Set the last n values to the last value
            if main_start_pt:
                last_value = arm._goal.trajectory.points[pos_idx - 1].positions[key]
            else:
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

    def trajectory_from_file(self, file_path, delay=0.1):
        joint_angles = []

        left_index = [self.FIXED_POSITIONS.index(jn) for jn in self.left.JOINT_NAMES]
        right_index = []
        if not self.single_arm:
            right_index = [self.FIXED_POSITIONS.index(jn) for jn in self.right.JOINT_NAMES]
        angle_index = left_index + right_index

        with open(file_path, 'r') as f:
            for line in f.readlines():
                line = line.strip('position: ').strip('[').strip(']')

                positions = np.fromstring(line, sep=',')
                # Sanity check
                if positions.size == 12:
                    joint_angles.append(positions[angle_index])

        self.left.init_trajectory()

        if not self.single_arm:
            self.right.init_trajectory()

        for pts in joint_angles:
            self.left.add_traj_point(pts[:6], delay)
            if not self.single_arm:
                self.right.add_traj_point(pts[6:], delay)

    def trajectory_from_bag(self, bag_file_path, start_idx=0, end_idx=None, use_recorded_time=True, delay=0.1, sample_points=0):
        if use_recorded_time and sample_points:
            rospy.logerr("Recored time and sample points cannot be used together")

        left_index = [self.FIXED_POSITIONS.index(jn) for jn in self.left.JOINT_NAMES]
        right_index = []
        if not self.single_arm:
            right_index = [self.FIXED_POSITIONS.index(jn) for jn in self.right.JOINT_NAMES]
        angle_index = left_index + right_index

        self.left.init_trajectory()
        if not self.single_arm:
            self.right.init_trajectory()

        count = 0
        prev_time = 0
        sample_count = 0
        with rosbag.Bag(bag_file_path, 'r') as bag:
            for _, msg, t in bag.read_messages():
                if count < start_idx:
                    continue
                if end_idx and count > end_idx:
                    break

                pos = np.array(msg.position)[[angle_index]]
                if use_recorded_time:
                    if not prev_time:
                        delay = .1
                    else:
                        delay = t.to_sec() - prev_time
                    prev_time = t.to_sec()

                if sample_points:
                    if sample_count != 0 and sample_count % sample_points:
                        sample_count += 1
                        continue
                    else:
                        sample_count += 1

                self.left.add_traj_point(pos[:6], delay)
                if not self.single_arm:
                    self.right.add_traj_point(pos[6:], delay)

                count += 1

    def trajectory_learning(self, points, delay=0.01):
        arms = [self.left, self.right]
        curr_angles = []

        for arm in arms:
            arm.init_trajectory()

        for arm in arms:
            curr_angles.append(arm.get_current_joint_angles())

        # Set the wrist1 and wrist3 joint values
        points = points.T
        points[:, JointNames.WRIST_1] = curr_angles[0][JointNames.WRIST_1]
        # points[:, JointNames.WRIST_3] = curr_angles[0][JointNames.WRIST_3]

        left_points = points.copy()
        left_points[:, JointNames.WRIST_3] = curr_angles[0][JointNames.WRIST_3]

        right_points = points.copy()
        right_points[:, JointNames.WRIST_3] = curr_angles[1][JointNames.WRIST_3]

        # Right pan joint is negative
        right_points[:, JointNames.SHOULDER_PAN] *= -1

        for lpt, rpt in zip(left_points, right_points):
            self.left.add_traj_point(lpt, delay)
            self.right.add_traj_point(rpt, delay)

    def interpolate(self, arm_str, points, delay=0.15, skip_joints=[]):
        if arm_str == 'left':
            arms = [self.left]

        if arm_str == 'right':
            arms = [self.right]

        current_angles = arms[0].get_current_joint_angles()
        for arm in arms:
            arm.init_trajectory()

        diff = np.array(points) - np.array(current_angles)
        max_idx = np.argmax(np.abs(diff))
        max_value = abs(diff[max_idx])

        steps = float(max_value) / 0.06
        joint_pos = []
        for idx in range(6):
            joint_pos.append(np.linspace(current_angles[idx], points[idx], steps))

        joint_pos = np.array(joint_pos).T

        if skip_joints:
            for idx in skip_joints:
                joint_pos[:, idx] = current_angles[idx]

        for pts in joint_pos:
            for arm in arms:
                arm.add_traj_point(pts, delay)

    def visualize_trajectory(self):
        if not self.display:
            return

        if not self.single_arm:
            self.adjust_traj_length()

        if self.rviz_pub.get_num_connections() < 1:
            rospy.logerr("Rviz topic not subscribed")
            return

        msg = DisplayTrajectory()
        msg.trajectory_start = self.robot_model.get_current_state()

        traj = RobotTrajectory()
        goal = JointTrajectory()

        goal.joint_names = self.left.JOINT_NAMES[:]
        if not self.single_arm:
            goal.joint_names += self.right.JOINT_NAMES[:]

        # Make the sim run slowly
        # delay = 1
        for idx in range(len(self.left._goal.trajectory.points)):
            comb_point = JointTrajectoryPoint()
            lpt = self.left._goal.trajectory.points[idx]

            comb_point.positions = lpt.positions[:]
            if not self.single_arm:
                comb_point.positions += self.right._goal.trajectory.points[idx].positions[:]

            comb_point.time_from_start = lpt.time_from_start
            # comb_point.time_from_start = rospy.Duration(idx * delay)
            goal.points.append(comb_point)

        traj.joint_trajectory = goal
        msg.trajectory.append(traj)

        duration = goal.points[-1].time_from_start.to_sec()
        rospy.loginfo("Waiting for trajectory animation {} seconds".format(duration))
        self.rviz_pub.publish(msg)
        rospy.sleep(duration)

    def print_joint_trajectory(self, traj):
        for pt in traj.points:
            print ["{:.4f}".format(i) for i in pt.positions]

    def print_numpy_array(self, pts):
        print ["{:.4f}".format(i) for i in pts]
