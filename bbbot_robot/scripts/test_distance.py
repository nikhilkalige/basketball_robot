#! /usr/bin/env python
import rospy
import sys
import numpy as np
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN
import time
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK


def get_ik_service(robot, pose, name, angles):
    msg = GetPositionIKRequest()
    msg.ik_request.group_name = name
    msg.ik_request.ik_link_name = 'leftarm_ee_link'
    # msg.ik_request.robot_state.joint_state.name = robot.left.JOINT_NAMES
    # msg.ik_request.robot_state.joint_state.position = angles
    #print pose
    pose.pose.position.z = 1.5
    #print pose

    msg.ik_request.pose_stamped = pose

    try:
        fk_service = rospy.ServiceProxy("compute_ik", GetPositionIK)
        resp = fk_service(msg)
        if resp.error_code.val != 1:
            print resp
            return -1
    except rospy.ServiceException as e:
        rospy.logwarn("Exception on fk service {}".format(e))
        return -1
    return resp


def main():
    rospy.init_node("basketball_robot_test")
    robot = Robot(use_prefix=True)
    # robot.control_torque(False)
    time.sleep(4)
    resp = robot.get_fk_service(robot.left.get_current_joint_angles(), robot.right.get_current_joint_angles(),
                                ['leftarm_ee_link', 'rightarm_ee_link'])
    print(resp.pose_stamped[0])
    print(resp.pose_stamped[0])
    return
    s = get_ik_service(robot, resp.pose_stamped[0], 'leftarm', robot.left.get_current_joint_angles())
    print(robot.left.get_current_joint_angles())
    print(s.solution.joint_state.position[:6])
    # robot.control_torque()
    # return
    # while True:
    #     print robot.get_dist_btw_effectors(robot.left.get_current_joint_angles(), robot.right.get_current_joint_angles())
    #     time.sleep(3)

if __name__ == '__main__':
    main()
