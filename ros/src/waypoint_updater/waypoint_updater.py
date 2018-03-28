#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import copy

import tf
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50# 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.traffic_waypoint = -1

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        if self.base_waypoints == None:
            return
        
        closest = self.get_closest_waypoint(msg.pose)

        waypoints = []
        for waypoint in self.base_waypoints.waypoints[closest:closest+LOOKAHEAD_WPS]:
            waypoints.append(copy.deepcopy(waypoint))

        if self.traffic_waypoint != -1:
            max_v = self.get_waypoint_velocity(self.base_waypoints.waypoints[closest])
            for i in range(len(waypoints)):
                if (i + closest) >= self.traffic_waypoint:
                    self.set_waypoint_velocity(waypoints, i, 0)
                else:
                    # XXX Very navie implementation. We have to improve it.
                    d = self.distance(self.base_waypoints.waypoints, i + closest, self.traffic_waypoint)
                    v = min(d * max_v / 30., max_v)
                    self.set_waypoint_velocity(waypoints, i, v)
            #rospy.logwarn('dist:{}, v:{}'.format(self.distance(self.base_waypoints.waypoints, closest, self.traffic_waypoint), self.get_waypoint_velocity(waypoints[0])))

        self.publish(waypoints)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def dist(self, p1, p2):
        return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)

    def get_closest_waypoint(self, pose):
        closest_len = float("inf")
        closest_waypoint_idx = 0
        for i, waypoint in enumerate(self.base_waypoints.waypoints):
            p1 = waypoint.pose.pose.position
            p2 = pose.position
            d = self.dist(p1, p2)
            if d < closest_len:
                closest_len = d
                closest_waypoint_idx = i
        return closest_waypoint_idx

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
