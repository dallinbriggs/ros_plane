#!/usr/bin/env python
# Python implementation of "path_planner.cpp"

import rospy
from ros_plane.msg import Waypoint
import math
import sys

lp1 = sys.argv[1]
lp2 = sys.argv[2]
aa = sys.argv[3]

def publishwaypoints(lp1, lp2, aa):

	# Init ROS Node
	rospy.init_node('landing_path_planner', anonymous=True)

	# Init Publisher
	waypointPublisher = rospy.Publisher('waypoint_path',Waypoint, queue_size=10)

	# Sleep, (this fixed bug of first waypoint not publishing)
	d = rospy.Duration(.5)
	rospy.sleep(d)

	land_point = [float(lp1), float(lp2)] # North, East (meters)
	approach_angle = float(aa) # chi in radians (East of North)

	land = Waypoint()

	# Setup land waypoint
	land.w[0] = land_point[0]
	land.w[1] = land_point[1]
	land.w[2] = 0.0
	land.chi_d = approach_angle
	land.Va_d = 0.0

	land.chi_valid = True # True
	land.set_current = False
	land.reset = False
	land.land = True # True to land now

	# Setup approach 1
	approach1 = Waypoint()
	approach1.w[0] = land_point[0] - math.cos(approach_angle)*30
	approach1.w[1] = land_point[1] - math.sin(approach_angle)*30
	approach1.w[2] = -5.0
	approach1.chi_d = approach_angle
	approach1.Va_d = 5.0
	approach1.chi_valid = True # True
	approach1.set_current = False
	approach1.reset = False
	approach1.land = True

	# Setup approach 2
	approach2 = Waypoint()
	approach2.w[0] = land_point[0] - math.cos(approach_angle)*60
	approach2.w[1] = land_point[1] - math.sin(approach_angle)*60
	approach2.w[2] = -10.0
	approach2.chi_d = approach_angle
	approach2.Va_d = 7.0
	approach2.chi_valid = True # True
	approach2.set_current = False
	approach2.reset = False
	approach2.land = True

	# Setup approach 3
	approach3 = Waypoint()
	approach3.w[0] = land_point[0] - math.cos(approach_angle)*90
	approach3.w[1] = land_point[1] - math.sin(approach_angle)*90
	approach3.w[2] = -15.0
	approach3.chi_d = approach_angle
	approach3.Va_d = 7.0
	approach3.chi_valid = True # True
	approach3.set_current = False
	approach3.reset = False
	approach3.land = False

	# Setup approach 4
	approach4 = Waypoint()
	approach4.w[0] = land_point[0] - math.cos(approach_angle)*160
	approach4.w[1] = land_point[1] - math.sin(approach_angle)*160
	approach4.w[2] = -33.0
	approach4.chi_d = approach_angle
	approach4.Va_d = 15.0
	approach4.chi_valid = True # True
	approach4.set_current = False
	approach4.reset = True
	approach4.land = False

	# Overshoot
	land_p1 = Waypoint()
	land_p1.w[0] = land_point[0] + math.cos(approach_angle)*100
	land_p1.w[1] = land_point[1] + math.sin(approach_angle)*100
	land_p1.w[2] = 0.0
	land_p1.chi_d = approach_angle
	land_p1.Va_d = 0.0

	land_p1.chi_valid = True # True
	land_p1.set_current = False
	land_p1.reset = False
	land_p1.land = True # True to land now

	# Overshoot2
	land_p2 = Waypoint()
	land_p2.w[0] = land_point[0] + math.cos(approach_angle)*1000
	land_p2.w[1] = land_point[1] + math.sin(approach_angle)*1000
	land_p2.w[2] = 0.0
	land_p2.chi_d = approach_angle
	land_p2.Va_d = 0.0

	land_p2.chi_valid = True # True
	land_p2.set_current = False
	land_p2.reset = False
	land_p2.land = True # True to land now

	# Overshoot2
	land_p3 = Waypoint()
	land_p3.w[0] = land_point[0] + math.cos(approach_angle)*1100
	land_p3.w[1] = land_point[1] + math.sin(approach_angle)*1100
	land_p3.w[2] = 0.0
	land_p3.chi_d = approach_angle
	land_p3.Va_d = 0.0

	land_p3.chi_valid = True # True
	land_p3.set_current = False
	land_p3.reset = False
	land_p3.land = True # True to land now

	waypoints = [approach4, approach3, approach2, approach1, land, land_p1, land_p2, land_p3]
	# print land.Va_d, approach1.Va_d, approach3.Va_d

    # Loop through each waypoint
	for wp in waypoints:

		# Publish the Waypoint
		waypointPublisher.publish(wp)

		# Sleep
		d = rospy.Duration(0.5)
		rospy.sleep(d)


if __name__ == '__main__':

	# Just run the publisher once
	try:
		publishwaypoints()
	except rospy.ROSInterruptException:
		pass
