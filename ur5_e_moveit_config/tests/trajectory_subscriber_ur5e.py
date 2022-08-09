#!/usr/bin/env python
"""
    Modified by haavar from pickandplace tutorial in robotics hub
    Subscribes to MyPublisher topic.
    Uses MoveIt to compute a trajectory from the target to the destination.
    Trajectory is then published to PickAndPlaceTrajectory topic.

"""
import rospy

from ur5_e_moveit_config.msg import MyMsg
from ur_msgs.msg import MyMsg


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard:\n%s", data)

def listener():
    rospy.init_node('MyMsg_Subscriber', anonymous=True)
    rospy.Subscriber("/ur5e_joints", MyMsg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
