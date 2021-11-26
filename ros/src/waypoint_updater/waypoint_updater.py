#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        
        self.loop()

    # Define a loop that controls the publishing frequency, targeting 50 Hz
    def loop(self):
        # Set a publishing rate of 50 Hz
        rate = rospy.Rate(50)
        # Check that Rospy is not shutdown and that we have both the pose and the base waypoints
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                # Get the closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_id()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()
    
    def get_closest_waypoint_id(self):
        # Get the coordinates of our car
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # Return the index of a coordinate that is the closest point in our KDTree
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Use a hyperplane to see if the closest point we have is ahead of the car
        # Get the two closest coordinates from waypoints 2D
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        # Compute a dot product and check if it is positive or negative, to see if the
        # closest coodinate is in front of the car or not
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        # If the waypoint is behind us, take the net one
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        # The message type needs to be a lane, so we create a new lane object
        lane = Lane()
        lane.header = self.base_waypoints.header
        # The waypoints for that lane should be the waypoints from your base waypoints, sliced
        # from the closest index to the closest index plus the number of waypoints chosen
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        # TODO: Implement
        # Store the car's pose
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # Store in the object the waypoints that the callback receives as argument
        self.base_waypoints = waypoints
        # Check that waypoints_2d is initialized before calling the subscriber
        if not self.waypoints_2d:
            # Build a list of 2D coordinates for each waypoint
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            # KDTree is a data structure that allows you to look up the closest points in space really efficiently
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
