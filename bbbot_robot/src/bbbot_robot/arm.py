import sys
import rospy
import actionlib
from manipulator_actions.msg import ControlTorqueGoal, ControlTorqueAction
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import GoalStatus


class Arm:
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    def __init__(self, name="", namespace="", use_prefix=False, sim=False):
        self.name = name
        self.ns = namespace

        if use_prefix:
            self.JOINT_NAMES = ['{}_{}'.format(self.ns, name) for name in self.JOINT_NAMES]

        self.trajectory_active = False

        self.torque_client = actionlib.SimpleActionClient(
            "{}/control_torque".format(self.ns), ControlTorqueAction)
        self.traj_client = actionlib.SimpleActionClient(
            "{}/joint_trajectory_controller/follow_joint_trajectory".format(self.ns),
            FollowJointTrajectoryAction)

        rospy.on_shutdown(self.cleanup)

        self.logmsg("Connecting to the clients")
        log_str = "Timed out connecting to {} server"

        if not sim:
            if not self.torque_client.wait_for_server(timeout=rospy.Duration(10)):
                rospy.logerr(log_str.format("Control Torque"))
                rospy.signal_shutdown("Unable to connect to server")
                sys.exit(1)

        if not self.traj_client.wait_for_server(timeout=rospy.Duration(10)):
            rospy.logerr(log_str.format("Joint Trajectory"))
            rospy.signal_shutdown("Unable to connect to server")
            sys.exit(1)

        self.logmsg("Succesfully connected to both clients")
        self._goal = FollowJointTrajectoryGoal()

    def logmsg(self, msg=""):
        rospy.loginfo("{}:- {}".format(self.name, msg))

    def control_torque(self, enable=True):
        goal = ControlTorqueGoal()
        goal.joint_names = self.JOINT_NAMES
        goal.enable = [enable] * len(self.JOINT_NAMES)

        self.torque_client.send_goal(goal)
        self.torque_client.wait_for_result()

        result = self.torque_client.get_result()
        if not result:
            rospy.logerr(
                "Torque {} failed".format("enable" if enable else "disable"))
        else:
            self.logmsg(
                "Torque {} successfully".format("enabled" if enable else "disabled"))
        return result.success

    def get_current_joint_angles(self):
        msg = rospy.wait_for_message(
            "{}/joint_states".format(self.ns), JointState)
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

    def add_traj_point(self, joint_angles, time_after_last_point=1):
        point = JointTrajectoryPoint()
        if len(joint_angles) < len(self.JOINT_NAMES):
            rospy.logerr('Unable add point, invalid number of joint angles')
            return
        point.positions = joint_angles[:]
        self._time_since_start = self._time_since_start + time_after_last_point
        point.time_from_start = rospy.Duration(self._time_since_start)

        self._goal.trajectory.points.append(point)

    def send_trajectory(self, start_time):
        if self.trajectory_active:
            return False

        self._goal.trajectory.header.stamp = start_time
        # print self._goal.trajectory.header
        self.traj_client.send_goal(self._goal, done_cb=self.traj_done_callback)

        self.trajectory_active = True
        self.logmsg("Trajectory with {} points sent".format(
            len(self._goal.trajectory.points)))
        return True

    def traj_done_callback(self, goal_status, result):
        if goal_status == GoalStatus.SUCCEEDED:
            self.logmsg("Trajectory successfully executed")
        else:
            self.logmsg(
                "Trajectory failed to execute with status: {}".format(goal_status))
            sys.exit(1)
        self.logmsg(result)
        self.trajectory_active = False

    def cleanup(self):
        try:
            status = self.traj_client.get_state()
        except Exception:
            self.logmsg("No active goal found")
            return
        if status == GoalStatus.ACTIVE:
            self.logmsg("Cancelling all goals")
            self.traj_client.cancel_all_goals()

    def echo_trajectory(self):
        self.logmsg("No of points: {}".format(len(self._goal.trajectory.points)))
        for pt in self._goal.trajectory.points:
            print ["{:.7f}".format(i) for i in pt.positions], '\t', 'Time: ', pt.time_from_start
