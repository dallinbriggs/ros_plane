#!/usr/bin/env python
# Python implementation of "path_planner.cpp"

import rospy
from ros_plane.msg import Waypoint
import math
import new_RRT
import numpy as np
from current_path_generator import get_full_current_path
from PointInBounds import *

# num_waypoints = 6

def publishwaypoints():

	# Init ROS Node
	rospy.init_node('new_rrt_path_plan', anonymous=True)

	init_lat = 40.174477
	init_lon = -111.651655
	init_alt = 6.37

	EARTH_RADIUS = 6370027.0

	wp_file = ('example_wps.txt')

	NED_waypoints=[]

	with open(wp_file) as f:
		for line in f:
			latlon = line.split(' ')
			lat = float(latlon[0])
			lon = float(latlon[1])
			alt = float(latlon[2])
			chi = float(latlon[3])
			# print lat
			# print lon
			N = EARTH_RADIUS*(lat - init_lat)*np.pi/180.0;
			E = EARTH_RADIUS*cos(lat*np.pi/180.0)*(lon - init_lon)*np.pi/180.0;
			D = -1*(alt*0.3048 - init_alt)
			NED_waypoints.append([N,E,D])

	print "\n\n", NED_waypoints

	# List of waypoints from file
	wp1 = [500, -200, -60, 0.0]
	wp2 = [200, 200, -60, 1.2]

	NED_waypoints = [wp1, wp2]
	checker = pointGood()

	wps_to_add = []

	for i in range(0,len(NED_waypoints)-1):

		points = get_full_current_path([NED_waypoints[i],NED_waypoints[i+1]])

		valid_path = True

		for point in points:
			good = checker.check_point(point[0], point[1])
			# print "checked point"
			if not good:
				valid_path = False
				break

		if not valid_path:
			print "Collision on index: ", i

			wpp_pos = np.array([[NED_waypoints[i][0],NED_waypoints[i][1],NED_waypoints[i][2]]]).T
			wpp_start = [wpp_pos, NED_waypoints[i][3], 15.0]
			start_node = new_RRT.rrtNode(0,None,wpp_start)

			wpp_pos = np.array([[NED_waypoints[i+1][0],NED_waypoints[i+1][1],NED_waypoints[i+1][2]]]).T
			wpp_end = [wpp_pos, NED_waypoints[i+1][3], 15.0]
			end_node = new_RRT.rrtNode(0, None, wpp_end)
			rr = new_RRT.RRT(wpp_start, wpp_end)
			# path_valid = rr.check_dubins_path(start_node,end_node)

			# t_past = rospy.Time.now()
			path = rr.find_path()
			# rospy.logwarn(rospy.Time.now()-t_past)

			# print len(path)-2
			# for k in range(0, len(path)-2):
			# 	index = len(path) - 2 - k
			# 	for _ in range(0,len(path[index])):
			# 		wps_rrt.insert(i*5+5,path[index][len(path[index])-1-_])

			# print path
			added_wps = path[1:len(path)-1]
			with_index = [i, added_wps]
			wps_to_add.insert(0, with_index)

			# print wps_rrt
		else:
			print 'OK index: ', i
	# print "\nwps_to add ", wps_to_add

	for i in range(0,len(wps_to_add)):
		index = wps_to_add[i][0]
		wps = wps_to_add[i][1]
		print len(wps)
		for j in range(len(wps)-1,-1,-1):
			# print "for:",j
			NED_waypoints.insert(index+1,wps[j])
			# print "added", wps[j]
		# NED_waypoints.insert()


	# num_rrt_wps = len(wps_rrt)/5
	# Loop through each waypoint
	print"\n\n",NED_waypoints
	for i in range(0,num_rrt_wps):

		# Make waypoint a Waypoint msg
		new_waypoint = Waypoint()

		new_waypoint.w[0] = wps_rrt[i*5 + 0]
		new_waypoint.w[1] = wps_rrt[i*5 + 1]
		new_waypoint.w[2] = wps_rrt[i*5 + 2]
		new_waypoint.chi_d = wps_rrt[i*5 + 3]

		new_waypoint.chi_valid = True # True
		new_waypoint.set_current = False
		new_waypoint.Va_d = wps_rrt[i*5 + 4]
		new_waypoint.reset = False
		new_waypoint.land = False
		new_waypoint.drop = False
		new_

		# Publish the Waypoint
		waypointPublisher.publish(new_waypoint)
		rospy.logwarn('waypoint published')

		# Sleep
		d = rospy.Duration(0.5)
		rospy.sleep(d)


if __name__ == '__main__':
	# Just run the publisher once
	try:
		publishwaypoints()
	except rospy.ROSInterruptException:
		pass
