#! /usr/bin/env python
import roslib
import rospy
import actionlib
from manipulator_actions.msg import ControlTorqueAction, ControlTorqueGoal


roslib.load_manifest('manipulator_driver')


def control_torqure_action():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient(
        '/rightarm/control_torque', ControlTorqueAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("Waiting for the server")
    client.wait_for_server()
    print("Server Found")

    # Creates a goal to send to the action server.
    goal = ControlTorqueGoal()
    goal.joint_names = ['rightarm_wrist_3_joint']
    goal.enable = [False]

    # Sends the goal to the action server.
    client.send_goal(goal)

    print("Send goal")
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('control_torque_test_client')
        result = control_torqure_action()
        print result
        print "Result: Torque command ", "succeeded" if result.success else "failed"
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
