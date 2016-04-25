import sys
import rospy
import actionlib
from manipulator_actions.msg import ControlTorqueGoal, ControlTorqueAction
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import GoalStatus


class Robot:
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    def __init__(self):
        self.torque_client = actionlib.SimpleActionClient(
            'control_torque', ControlTorqueAction)
        self.traj_client = actionlib.SimpleActionClient(
            'pos_based_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        rospy.on_shutdown(self.cleanup)

        rospy.loginfo("Connecting to the clients")
        log_str = "Timed out connecting to {} server"

        if not self.torque_client.wait_for_server(timeout=rospy.Duration(10)):
            rospy.logerr(log_str.format("Control Torque"))
            rospy.signal_shutdown("Unable to connect to server")
            sys.exit(1)

        if not self.traj_client.wait_for_server(timeout=rospy.Duration(10)):
            rospy.logerr(log_str.format("Joint Trajectory"))
            rospy.signal_shutdown("Unable to connect to server")
            sys.exit(1)

        rospy.loginfo("Succesfully connected to both clients")
        self._goal = FollowJointTrajectoryGoal()

    def control_torque(self, enable=True):
        goal = ControlTorqueGoal()
        goal.joint_names = self.JOINT_NAMES
        goal.enable = [enable] * len(self.JOINT_NAMES)

        self.torque_client.send_goal(goal)
        self.torque_client.wait_for_result()

        result = self.torque_client.get_result()
        if not result:
            rospy.logerr("Torque {} failed".format("enable" if enable else "disable"))
        else:
            rospy.loginfo("Torque {} successfully".format("enabled" if enable else "disabled"))
        return result.success

    def get_current_joint_angles(self):
        msg = rospy.wait_for_message('joint_states', JointState)
        positions = []
        for joint in self.JOINT_NAMES:
            positions.append(msg.position[msg.name.index(joint)])
        return positions

    def init_trajectory(self, add_starting_point=True, time_from_start=0):
        self._goal = FollowJointTrajectoryGoal()
        self._time_since_start = time_from_start
        self._goal.trajectory.joint_names = self.JOINT_NAMES

        if add_starting_point:
            current_pos = self.get_current_joint_angles()
            self.add_traj_point(current_pos)

    def add_traj_point(self, joint_angles, time_after_last_point=0):
        point = JointTrajectoryPoint()
        if len(joint_angles) < len(self.JOINT_NAMES):
            rospy.logerr('Unable add point, invalid number of joint angles')
            return
        point.positions = joint_angles[:]
        self._time_since_start = self._time_since_start + time_after_last_point
        point.time_from_start = rospy.Duration(self._time_since_start)

        self._goal.trajectory.points.append(point)

    def send_trajectory(self):
        self._goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1)
        # print self._goal.trajectory.header
        self.traj_client.send_goal(self._goal)
        rospy.loginfo("Trajectory with {} points sent".format(len(self._goal.trajectory.points)))

        self.traj_client.wait_for_result()
        status = self.traj_client.get_state()
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Trajectory successfully executed")
        else:
            rospy.loginfo("Trajectory failed to execute with status: {}".format(status))
        return self.traj_client.get_result()

    def cleanup(self):
        try:
            status = self.traj_client.get_state()
        except Exception:
            rospy.loginfo("No active goal found")
            return
        if status == GoalStatus.ACTIVE:
            rospy.loginfo("Cancelling all goals")
            self.traj_client.cancel_all_goals()
