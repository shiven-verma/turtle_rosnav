#!/usr/bin/env python
import tf
import numpy as np
import rospy
import turtlesim.msg
from sensor_msgs.msg import LaserScan
from turtlesim.srv import Spawn
from turtlesim.srv import SpawnRequest
from turtlesim.srv import SpawnResponse
from turtlesim.msg import Pose
from math import sqrt, pow, atan2, asin, cos
# this node has to generate turtles at some postions to serve as obstacles to the main turtlesim node.
# it should also publish a laser_scan msg to a topic subscribed by the navigation stack
# that contains the information about the location of the obstacles.


class Generator:

    def __init__(self):
        rospy.init_node("obstacle_generator")
        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        self.spawnrange = [[4, 2, 0, "turtle2"], [
            8, 5, 0, "turtle3"], [9, 8, 0, "turtle4"]]
        for i in range(0, 3):
            spawner(self.spawnrange[i][0], self.spawnrange[i][1],
                    self.spawnrange[i][2], self.spawnrange[i][3])
        self.pose_subscribera = rospy.Subscriber(
            '/turtle1/pose', turtlesim.msg.Pose, self.update_pose)
        self.scan_pub = rospy.Publisher(
            'scan', LaserScan, queue_size=50)
        self.pose = turtlesim.msg.Pose()

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""

        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 4)

    def obstaclegen(self):

        num_readings = 100
        laser_frequency = 40

        count = 0
        r = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            self.scan = LaserScan()

            self.scan.header.stamp = current_time
            self.scan.header.frame_id = 'base_link'
            self.scan.angle_min = -1.57
            self.scan.angle_max = 1.57
            self.scan.angle_increment = 3.14 / num_readings
            self.scan.time_increment = (1.0 / laser_frequency) / (num_readings)
            self.scan.range_min = 0.0
            self.scan.range_max = 100.0
            self.radius = 0.5
            self.scan.ranges = []
            rangetemp = np.zeros((num_readings, 3))
            distance = np.zeros(3)
            dcircum = np.zeros(num_readings)
            ang = np.zeros(3)
            for j in range(0, 3):
                # distance between centres
                distance[j] = sqrt(pow(
                    (self.pose.x-self.spawnrange[j][0]), 2)+pow((self.pose.y-self.spawnrange[j][1]), 2))
                # angle between centres and horizontal
                ang[j] = atan2(self.spawnrange[j][1]-self.pose.y,
                               self.spawnrange[j][0]-self.pose.x)-self.pose.theta
                thetas = np.linspace(self.scan.angle_min,
                                     self.scan.angle_max, num_readings, endpoint=False)
                thetas = thetas-ang[j]
                theta_dist = np.power(distance[j]*np.cos(thetas), 2) +\
                    pow(self.radius, 2)-pow(distance[j], 2)
                dcircum[:] = float('inf')
                dcircum[theta_dist > 0] = distance[j] * \
                    np.cos(thetas[theta_dist > 0]) - \
                    np.sqrt(theta_dist[theta_dist > 0])
                rangetemp[:, j] = dcircum

            temp2 = np.amin(rangetemp, axis=1)
            for counter_var in range(0, num_readings):
                self.scan.ranges.append(temp2[counter_var])
            self.scan_pub.publish(self.scan)
            r.sleep()


if __name__ == '__main__':
    try:
        x = Generator()
        x.obstaclegen()
    except rospy.ROSInterruptException:
        pass
