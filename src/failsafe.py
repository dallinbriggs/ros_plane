#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from rosflight_msgs.msg import Status


class failsafe():

	def __init__(self):
		self.new_failsafe = True
		self.new_failsafe_time = 0
		self.RTHNOW = False
		print "RTHNOW init to False"
		self.TERMINATENOW = False

		self.RTH = Bool()
		self.RTH.data = False
		print "RTH DATA INIT TO False"
		self.Terminate = Bool()
		self.Terminate.data = False

		self.fail_RTH = False
		self.fail_terminate = False

		#status_sub = rospy.Subscriber('status', Status, failsafe_callback)
		self.RTHNOW_sub = rospy.Subscriber('RTH_now', Bool, self.RTHNOW_callback)
		self.TERMINATENOW_sub = rospy.Subscriber('TERMINATE_now', Bool, self.TERMINATENOW_callback)
		self.RTH_pub = rospy.Publisher('RTH', Bool, queue_size=10)
		self.Terminate_pub = rospy.Publisher('terminate_flight', Bool, queue_size=10)

	def failsafe_callback(self, msg):
		if msg.failsafe: # If in Failsafe mode
				if self.new_failsafe:
					self.new_failsafe_time = msg.header.stamp
					self.new_failsafe = False
				else:
					self.time = self.new_fail_safe_time - msg.header.stamp
					if self.time > 30: # If in failsafe for more than 30s
						self.fail_RTH = True
					if self.time > 180: # If in failsafe for 3 mins
						self.fail_terminate = True
		else:
			self.fail_RTH = False
			self.fail_terminate = False
			self.new_failsafe = True

	def RTHNOW_callback(self, msg):
	    print"RTH NOW callback"
	    self.RTHNOW = msg.data
	    print "RTHNOW: ", self.RTHNOW

	def TERMINATENOW_callback(self, msg):
		print "TERMINATENOW callback"
		self.TERMINATENOW = msg.data


if __name__ == '__main__':
	rospy.init_node('Status Watcher')

	safe = failsafe()

	r = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		safe.RTH.data = safe.RTHNOW or safe.fail_RTH
		safe.Terminate.data = safe.TERMINATENOW or safe.fail_terminate
		safe.RTH_pub.publish(safe.RTH)
		# print "RTH pub: ", safe.RTH.data
		safe.Terminate_pub.publish(safe.Terminate)
		r.sleep()
