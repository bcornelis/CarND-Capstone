#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
	self.current_pose = None
	self.waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
	self.current_pose = msg.pose
	self.send_final_waypoints()

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

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

    def get_next_waypoint_index(self, pose):
	# proper state?
	if (self.waypoints is None or pose is None):
	    return None

	# TODO: this should be optimized keeping the last location as a parameter
	# car position
	car_x = pose.position.x
	car_y = pose.position.y

	# Values we're looking for
	closest_dist = None
	closest_wp_idx = -1;

	for waypoint_idx, waypoint in enumerate(self.waypoints):
	    # easier references
	    waypoint_x = waypoint.pose.pose.position.x
	    waypoint_y = waypoint.pose.pose.position.y

	    # calculate the distance to this waypoint
	    dist = math.sqrt((car_x-waypoint_x)**2 + (car_y-waypoint_y)**2)

	    if (closest_dist is None or closest_dist > dist):
		closest_dist = dist
		closest_wp_idx = waypoint_idx
	
	# ----------
	rospy.logdebug('Closest waypoint for current position (%d,%d) is waypoint %d with position (%d,%d)', 
		pose.position.x, pose.position.y,
		closest_wp_idx,
		self.waypoints[closest_wp_idx].pose.pose.position.x, self.waypoints[closest_wp_idx].pose.pose.position.y )
	# ----------
  	return closest_wp_idx if (closest_dist is not None) else None

    def send_final_waypoints(self):
	# Make sure the required data is available
	if( self.current_pose is None or self.waypoints is None):
	    return

	# Get information about the cars current position
	next_waypoint_idx = self.get_next_waypoint_index(self.current_pose)

	# Generate an array containing the waypoints
	waypoints_in_front = self.waypoints[next_waypoint_idx:next_waypoint_idx+LOOKAHEAD_WPS]

	# set the velocities for those waypoints
	for waypoint_idx, waypoint in enumerate(waypoints_in_front):
	    self.set_waypoint_velocity( waypoints_in_front, waypoint_idx, 10*0.447)

	# Create the lane object to be send
	lane = Lane()
	lane.waypoints = waypoints_in_front

	self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
