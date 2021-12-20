#!/usr/bin/env python
import tf
import rospy
import roslib
from math import sin, cos
import time
# from turtlesim.msg import Pose
import turtlesim.msg    # for turtle pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber(
            '/turtle1/pose', turtlesim.msg.Pose, self.update_pose)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

        self.pose = turtlesim.msg.Pose()
        self.rate = rospy.Rate(20)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""

        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 4)

    def actualwork(self):
        current_time = rospy.Time.now()
        last_time = rospy.Time.now()
        r = rospy.Rate(60)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            self.odom_broadcaster = tf.TransformBroadcaster()
            self.odom_quat = tf.transformations.quaternion_from_euler(
                0, 0, self.pose.theta)
            self.odom_broadcaster.sendTransform(
                (self.pose.x, self.pose.y, 0), self.odom_quat, rospy.Time.now(), "base_link", "odom")   # edited slightly
            self.odom = Odometry()
            self.odom.header.stamp = current_time
            self.odom.header.frame_id = "odom"
            self.odom.pose.pose = Pose(
                Point(self.pose.x, self.pose.y, 0.), Quaternion(*self.odom_quat))
            self.odom.child_frame_id = "base_link"
            self.odom.twist.twist = Twist(
                Vector3(self.pose.linear_velocity*cos(self.pose.theta), self.pose.linear_velocity*sin(self.pose.theta), 0), Vector3(0, 0, self.pose.angular_velocity))  # maybe remove
            self.odom_pub.publish(self.odom)

            last_time = current_time
            r.sleep()


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.actualwork()
    except rospy.ROSInterruptException:
        pass
