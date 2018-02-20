#!/usr/bin/env python

import sys,serial
import threading
import __future__
from serial.tools import list_ports
import math
import time
import tf
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

current_angle = 0.0
desired_angle = 0.0
linear = 0.0
angular = 0.0
distance = 0.0
headingDone = 0
count = 0
countLimit = 20
checkAgain = 0
error = 15
distance_limit = 3	#in meters

def callback1(data):
	global current_angle
	current_angle = data.data

def callback2(data):
	global desired_angle
	desired_angle = data.data

def callback3(data):
	global distance
	distance = data.data

def publishVelocity(linear_vel,angular_vel):
	msg = Twist()
	msg.linear.x = linear_vel
        msg.angular.z = angular_vel
	cmd_pub.publish(msg)

def getParam():
	global linear
	global angular
	linear = rospy.get_param("/robot/linear_velocity")
	angular = rospy.get_param("/robot/angular_velocity")


def setHeading():
	global desired_angle
	global linear
	global angular
	global current_angle
	global distance
	global headingDone
	global count
	global checkAgain
	global countLimit
	global error
	global distance_limit

	diff = desired_angle - current_angle
        diff = math.radians(diff)
        theta = math.atan2(math.sin(diff),math.cos(diff))

	desired_lat = rospy.get_param("/gps_waypoint/desired_lat")
        desired_lon = rospy.get_param("/gps_waypoint/desired_lon")

	if ((desired_lat == 0) and (desired_lon == 0)):
		return

	if (checkAgain):
		if (((desired_angle+error)>current_angle) and ((desired_angle-error)<current_angle)):
			publishVelocity(0.0,0.0)		#stop
			headingDone = 1
			print("current: "+ str(current_angle) + "   desired: " + str(desired_angle) + "   stop")
		elif (theta < 0):
			publishVelocity(0.0, angular)	#move left
			headingDone = 0
			print("current: "+ str(current_angle) + "   desired: " + str(desired_angle) + "   move left")
		elif (theta >= 0):
			publishVelocity(0.0, -1*angular)		#move right
			headingDone = 0
			print("current: "+ str(current_angle) + "   desired: " + str(desired_angle) + "   move right")

	if (headingDone):
		count = count + 1
		print (str(count))
	else:
		count = 0

	if ((((desired_angle+error)>current_angle) and ((desired_angle-error)<current_angle)) != 1):
		checkAgain = 1
	else:
		headingDone = 1
		checkAgain = 0

	if (headingDone and (count >= countLimit) and (distance>distance_limit)):
		publishVelocity(linear, 0.0)		#move forward
		#headingDone = 0
		print("-------move forward--------" + str(count))




if __name__ == '__main__':
    try:
        rospy.init_node('set_heading')
        cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        rospy.Subscriber("/current_angle", Float32, callback1)
        rospy.Subscriber("/desired_angle", Float32, callback2)
	rospy.Subscriber("/distance", Float32, callback3)
	
	getParam()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		setHeading()
		rate.sleep()

    except KeyboardInterrupt:
        print "Key-interrupt"
        sys.exit(0)

    sys.exit(0)

