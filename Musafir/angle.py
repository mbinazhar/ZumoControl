#!/usr/bin/env python

import sys
import math
import rospy
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String, Float32
import tf
import time


def callbackIMU(data):
        imu_data = data.magnetic_field
        heading = math.atan2(data.magnetic_field.y, data.magnetic_field.x)

        if(heading < 0):
                heading += 2*math.pi
        if(heading > 2*math.pi):
                heading -= 2*math.pi
	headingAngle = (heading * 180)/(math.pi)
	headingAngle = math.fabs(headingAngle - 360)
        currentPub.publish(headingAngle)

def callbackGPS(data):
	#current latitude and longitude from gps
	lat1 = data.latitude
	lon1 = data.longitude
	#waypoint latitute and longitude
	lat2 = rospy.get_param("/gps_waypoint/desired_lat")
	lon2 = rospy.get_param("/gps_waypoint/desired_lon")
	
	print (str(lat2))
	
	dist_calc=0.0
	dist_calc2=0.0
	dLon = 0.0
	dLat = 0.0

	#convert current latitude and longitude to radians
  	lat1 = math.radians(lat1)
  	lon1 = math.radians(lon1)

	#convert waypoint latitude and longitude to radians
	lat2 = math.radians(lat2)
	lon2 = math.radians(lon2)

	dLat=(lat2-lat1)
  	dLon=(lon2-lon1)

	#finding distance
	dist_calc = (math.sin(dLat/2.0)*math.sin(dLat/2.0))
  	dist_calc2 = math.cos(lat1)
  	dist_calc2 = dist_calc2*math.cos(lat2)
  	dist_calc2 = dist_calc2*math.sin(dLon/2.0)
  	dist_calc2 = dist_calc2*math.sin(dLon/2.0)
  	dist_calc = dist_calc+dist_calc2
  	dist_calc =(2*math.atan2(math.sqrt(dist_calc),math.sqrt(1.0-dist_calc)))
	dist_calc = dist_calc*6371000.0  #Converting to meters

	#finding angle
	y = math.sin(dLon) * math.cos(lat2)
	x = (math.cos(lat1) * math.sin(lat2)) - (math.sin(lat1) * math.cos(lat2) * math.cos(dLon)) 
	heading = math.atan2(y, x)

	if (heading<0):
		heading = heading + 2*math.pi
	if (heading>(2*math.pi)):
		heading = heading - 2*math.pi

	heading = math.degrees(heading)

	desiredPub.publish(heading)
	distancePub.publish(dist_calc)

if __name__ == '__main__':
        try:
		rospy.init_node('imuAngle')
        	currentPub = rospy.Publisher("/current_angle", Float32, queue_size=1)
        	desiredPub = rospy.Publisher("/desired_angle", Float32, queue_size=1)
        	distancePub = rospy.Publisher("/distance", Float32, queue_size=1)
        	rate = rospy.Rate(10)
        	subIMU = rospy.Subscriber("/imu/mag", MagneticField, callbackIMU)
		subGPS = rospy.Subscriber("/gps/fix", NavSatFix, callbackGPS)
        	rospy.spin()

	except KeyboardInterrupt:
        	print "Key-interrupt"
        	sys.exit(0)
	
	sys.exit(0)
