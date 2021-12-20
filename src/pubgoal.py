#!/usr/bin/env python
import tf
import rospy
import roslib
from geometry_msgs.msg import PoseStamped
from turtlesim.msg import Pose


def talker():
    rospy.init_node("mynode")

    goal_publisher = rospy.Publisher(
        "move_base_simple/goal", PoseStamped, queue_size=5)
    goal = PoseStamped()
    rate = rospy.Rate(20)

    goal.header.seq = 1
    goal.header.stamp = rospy.Time(0)
    goal.header.frame_id = "odom"

    goal.pose.position.x = 2.67
    goal.pose.position.y = 0.67
    goal.pose.position.z = 0.0

    # goal.pose.position.x = 9.4
    # goal.pose.position.y = 5
    # goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        goal_publisher.publish(goal)
        goal.header.seq += 1
        goal.header.stamp = rospy.Time(0)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()

    except rospy.ROSInterruptException:
        pass
