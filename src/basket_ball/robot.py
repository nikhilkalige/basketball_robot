from .arm import Arm
import rospy
import numpy as np


class Robot:
    def __init__(self):
        self.left = Arm('LeftArm', 'leftarm')
        self.right = Arm('RightArm', 'rightarm')

    def control_torque(self, enable=True):
        if not self.left.control_torque():
            return False
        if not self.right.control_torque():
            return False
        return True

    def create_trajectory(self, arm_str="", index, end_point, increment, time=1):
        arm = getattr(self, arm_str)
        arm.init_trajectory()
        current_angles = arm.get_current_joint_angles()

        pts = self.generate_points(current_angles, index, end_point, increment)
        for pt in pts:
            robot.right.add_traj_point(pt, time)

    def generate_points(self, current_angles, index, end_point, increment):
        start_point = current_angles[index]
        points = []

        for pt in np.arange(start_point, end_point, increment):
            current_angles[index] = pt
            points.append(current_angles[:])

        rospy.loginfo("Generated angle values")
        for pt in points:
            print pt
        validate = raw_input("Validate generated angle values: \n")
        if 'n' in validate.lower():
            sys.exit(1)

        return points

    def start_trajectory(self, delay=0):
        start_time = rospy.Time.now() + rospy.Duration(delay)
        self.left.send_trajectory(start_time)
        self.right.send_trajectory(start_time)
