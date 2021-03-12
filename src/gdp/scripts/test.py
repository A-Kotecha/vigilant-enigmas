#!/usr/bin/env python

# GNC Node - communicates with the controller and the Task Allocation node

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64 #get Heading
from sensor_msgs.msg import NavSatFix #get GPS coordinates
from sensor_msgs.msg import Imu #Get IMU 
from sensor_msgs.msg import BatteryState #get Battery
#from gdp_centralised.msg import AgentInfo #System Architecture
from mavros_msgs.msg import State #FCU state
from mavros_msgs.msg import Waypoint # Waypoint format
from mavros_msgs.msg import WaypointReached # Waypoint reaching
from mavros_msgs.srv import CommandTOL # Take Off and Land
from mavros_msgs.srv import CommandHome #set Home Location
from mavros_msgs.srv import CommandBool #arming/disarming
from mavros_msgs.srv import SetMode # set flight mode
from mavros_msgs.srv import WaypointClear #Clear waypoint table
from mavros_msgs.srv import WaypointPush #Push new waypoint table
from gdp.msg import Order
from gdp.msg import OrderList
import time

#OrderList = []
#ConnectionAbortedError = []
#from rospy.numpy_msg import numpy_msg #TBC

# Global Variables
team = 'A'
idAgent = 1
ordersStack = [] # FIFO Stack of all the received orders

#Mission success flags
takeOffFlag = False
waypointFlag = False
preleaseFlag = False
releaseFlag = False
landFlag = False
processOrderFlag = True

####
def waypoint(coordinates): #coordinates = [lat, long, alt]

####################################### IMPLEMENT SECURITY FOR ALTITUDE (CHECK WITH THE GROUND ALT) ####################################
	global waypointFlag

	#Reset waypointFlag
	waypointFlag = False

	counter = 0
	wpPushFlag = False

	#Deliver a new mission plan to the autopilot

		#waypoint
	wp = Waypoint()
	wp.frame = 3 #Global frame:0 with relative alt: 3
	wp.command = 16 #Navigate to waypoint
	wp.is_current = True #TBC
	wp.autocontinue = True #TBC
	wp.param1 = 5 #Hold time is sec
	wp.param2 = 0.2 #Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
	wp.param3 = 0 #0 to pass through WP, else radius in meters to pass by WP
	wp.param4 = float('nan') #Desired yaw angle at waypoint (NaN for unchanged)
	wp.x_lat = coordinates[0]
	wp.y_long = coordinates[1]
	wp.z_alt = coordinates[2]

	#Push new waypoint
	while (wpPushFlag == False) and (counter <= 3):
		rospy.loginfo("Request new mission plan")
		rospy.wait_for_service('/mavros/mission/push')
		try:
			pushWaypointsRequest = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
			pushWaypointsResponse = pushWaypointsRequest(start_index = 0, waypoints = [wp])
			rospy.loginfo(pushWaypointsResponse)
			wpPushFlag = pushWaypointsResponse.success
		except rospy.ServiceException as e:
			rospy.loginfo("Request new mission plan failed: %s" %e)
		counter += 1

####

rospy.init_node('mavros_takeoff_python')
rate = rospy.Rate(10)

# Set Mode
print("\nSetting Mode")
rospy.wait_for_service('/mavros/set_mode')
try:
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    #response = change_mode(custom_mode="ALT_HOLD")
    response = change_mode(custom_mode="GUIDED")
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Set mode failed: %s" %e)

# Arm
print("\nArming")
rospy.wait_for_service('/mavros/cmd/arming')
try:
    arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    response = arming_cl(value = True)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Arming failed: %s" %e)
# 
# Takeoff
print("\nTaking off")
rospy.wait_for_service('/mavros/cmd/takeoff')
try:
    takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    response = takeoff_cl(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Takeoff failed: %s" %e)
# 
# print "\nHovering..."
time.sleep(8)
# 
####
waypoint([20,20,20])
time.sleep(8)
####
# Land
print("\nLanding")
rospy.wait_for_service('/mavros/cmd/land')
try:
    takeoff_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
    response = takeoff_cl(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Landing failed: %s" %e)