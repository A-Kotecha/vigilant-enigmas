#!/usr/bin/env python

# GNC Node - communicates with the controller and the Task Allocation node

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64 #get Heading
from sensor_msgs.msg import NavSatFix #get GPS coordinates
from nav_msgs.msg import Odometry #get GPS coordinates
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

from nav_msgs.msg import Odometry

#from gdp.msg import Order
#from gdp.msg import OrderList
import time
from pymavlink import mavutil, mavwp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal

#OrderList = []
#ConnectionAbortedError = []
#from rospy.numpy_msg import numpy_msg #TBC

'''
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ADD AGENT ID TO LOGINFO MESSAGES IF NEEDED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
'''


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


def meters2pos(x):
    return [10**-5*x[1],10**-5*x[0],x[2]]

class UAV():

	def __init__(self, idAgent):
		self.id = idAgent


	#Attributes

	'''
	position #GPS coordinates vector
	heading #Heading in deg
	IMU_acceleration #m/s2mode
	IMU_rate #rad/s
	velocity #m/s
	battery #percentage
	homeLocation #GPS coordinates vector
	state # [connected (bool), armed (bool), guided (bool), mode (string), system_status (uint8)]
	waypointReached #bool
	'''

	#Methods

	#Flight variables
	def getPosition(self, data):
		self.position = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z] #[deg, deg, m]
		print(self.position)

	def getHeading(self, data):
		self.heading = data #deg
                #print(self.heading)

	def getIMU(self, data):
		#self.IMU_acceleration = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z #m/s2
		self.IMU_rate = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z] #rad/s
                #print(self.IMU_rate)

	# def getVelocity(self, data): #Global or Local?
	# 	self.velocity_lin = ??
	# 	self.velocity_ang = ??

	def getBatteryStatus(self, data):
		self.battery = data.percentage #percentage
		#print("Checking Battery")
                #print(self.battery)

	def getState (self, data):
		self.state = [data.connected, data.armed, data.guided, data.mode, data.system_status]
		#print(self.state)
		#print("Getting State")

	#Autopilot
	def setHomeLocation(self, bool, data): #data = [lat, long, alt]
		self.homeLocation = data #[lat, long, alt]
		counter = 0
		flag = False
		while (flag == False) and (counter <= 3):
			rospy.loginfo("Request setHomeLocation")
			rospy.wait_for_service('/mavros/cmd/set_home')
			try:
				setHomeRequest = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
				setHomeResponse = setHomeRequest(current_gps = bool, latitude = self.homeLocation[0], longitude = self.homeLocation[1], altitude = self.homeLocation[2])
				rospy.loginfo(setHomeResponse)
				if setHomeResponse.result == self.homeLocation:
					flag = True
			except rospy.ServiceException as e:
				rospy.loginfo("SetHomePosition failed: %s" %e)
			counter += 1

		#send Message
		if flag == True:
			rospy.loginfo("SUCCESS: Set Home Location")
		else:
			rospy.loginfo("FAILURE: Set Home Location")


	def arm(self):

		counter = 0
		while (self.state[1] != True) and (counter <= 3):
			rospy.loginfo("Request arming")
			rospy.wait_for_service('/mavros/cmd/arming')
			try:
				armRequest = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
				armResponse = armRequest(value = True)
				rospy.loginfo(armResponse)
			except rospy.ServiceException as e:
				rospy.loginfo("Arming failed: %s" %e)
			counter += 1

		#send Message
		if self.state[1] == True:
			rospy.loginfo("SUCCESS: Arming")
		else:
			rospy.loginfo("FAILURE: Arming")

	def disarm(self):
		counter = 0
		while (self.state[1] != False) and (counter <= 3):
			rospy.loginfo("Request disarming")
			rospy.wait_for_service("/mavros/cmd/arming")
			try:
				disarmRequest = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
				disarmResponse = disarmRequest(value = False)
				rospy.loginfo(disarmResponse)
			except rospy.ServiceException as e:
				rospy.loginfo("Disarming failed: %s" %e)
			counter += 1

		#send Message
		if self.state[1] == False:
			rospy.loginfo("SUCCESS: Disarming")
		else:
			rospy.loginfo("FAILURE: Disarming")


	def setFlightMode(self, mode):
		'''
		mode is of type string
		ArduCopter flight modes: (https://ardupilot.org/copter/docs/flight-modes.html)
		STABILIZE
        GUIDED
		AUTO #Auto mode. Follow a pre-loaded mission plan. Preferred for normal mission.
		LOITER #Auto mode. Hover at current position and altitude.
		RTL #Return to home mode.
		LAND #Preferred for land.
		'''
		counter = 0

		while (self.state[3] != mode) and (counter <= 0):
			print("\nSetting Mode")
			rospy.loginfo("Request change flight mode to %s" %mode)
			rospy.wait_for_service('/mavros/set_mode')
			try:
				setModeRequest = rospy.ServiceProxy('/mavros/set_mode', SetMode)
				setModeResponse = setModeRequest(custom_mode = mode)
				rospy.loginfo(setModeResponse)
			except rospy.ServiceException as e:
				rospy.loginfo("Request change flight mode failed: %s" %e)

			counter += 1

		#send Message
		if self.state[3] == mode:
			rospy.loginfo("SUCCESS: Set Flight Mode: %s" %mode)
		else:
			rospy.loginfo("FAILURE: Set Flight Mode: %s" %mode)

	#Orders

	#TakeOff at current position and fly to a given altitude (relative)
	def takeOff(self):
		#Manual TakeOff
		self.setFlightMode('GUIDED')
		print("TKOFF")
		rospy.loginfo("Request TakeOff")
		rospy.wait_for_service('/mavros/cmd/takeoff')
		alt = 3 # Added just to make run, no idea where it is from. alt in takeOffResponse where here before
		try:
			takeOffRequest = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
			#takeOffResponse = takeOffRequest(altitude = self.position[2] + alt, latitude = self.position[0], longitude = self.position[1], min_pitch = 0, yaw = 0) #Absolute or relative altitude??
			takeOffResponse = takeOffRequest(altitude = alt, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
			rospy.loginfo(takeOffResponse)
			print("TakeOff end")
			#send message to task allocation
		except rospy.ServiceException as e:
			rospy.loginfo("Takeoff failed: %s" %e)
			#send message to Task Allocation

	def prelease(self):
		pass

	def NetAction (self):
		pass

	#Land at starting position
	def land(self):

		global landFlag
		landFlag = False

		self.setFlightMode(mode = 'RTL')
		#while landFlag == False:
		#	if on the ground:
		#		landFlag = True

		#disarm??
		#send message to Task Allocation

		'''
		#Manual Landing
		rospy.loginfo("Request Landing")
		rospy.wait_for_service('mavuav.setFlightMode(mode='STABILIZE')ros/cmd/land')
		try:
			landRequest = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
			landResponse = landRequest(altitude = self.altitude, latitude = self.position[0], longitude = self.position[1], min_pitch = 0, yaw = 0) #TBC altitude!!!
			rospy.loginfo(landResponse)
			#send message to task allocation
		except rospy.ServiceException as e:
			rospy.loginfo("Landing failed: %s" %e)
			#send message to Task Allocation
		'''

	def abortMission(self):
		#Switch to loiter modetakeOffResponse = takeOffRequest(altitude = self.position[2] + alt, latitude = self.position[0], longitude = self.position[1], min_pitch = 0, yaw = 0) #Absolute or relative altitude??
		rospy.loginfo("Request Abort Mission")
		self.setFlightMode(mode = 'LOITER')
		#Erase the waypoints table (mission plan)
		counter = 0
		clearFlag = False
		while (clearFlag == False) and (counter <= 3):
			rospy.loginfo("Request mission clearance")
			rospy.wait_for_service('/mavros/mission/clear')
			try:
				clearMissionRequest = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
				clearMissionResponse = clearMissionRequest()
				clearFlag = clearMissionResponse
			except rospy.ServiceException as e:
				rospy.loginfo("Mission clearance failed: %s" %e)

		#Empty the orders stack
		ordersStack = []

		#Send Message
		if clearFlag == True:
			rospy.loginfo("SUCCESS: Abort Mission")
		else:
			rospy.loginfo("FAILURE: Abort Mission")

	def go_to_defending(self,waypoints,vehicle):
		wp = []
		eps = 0.2
		for k in waypoints:
			wp.append(meters2pos(k))
		wp = np.array(wp)
		#wp = np.array(waypoints)
		for i in wp:
			point = LocationGlobalRelative(i[0], i[1], i[2]) #lon,lat,alt
			print(point)
			# Break and return from function just below target altitude.
			vehicle.simple_goto(point)
			time.sleep(1)


##End classNavSatFix


#Other parameters related to the mission


def waypointReached(data): #returns the index number of the reached waypoint
	#We assume that there is only one waypoint in the mission table at the same time
	global waypointFlag
	waypointFlag = True

##INITIALISATION

def initialisation():
	pass

class ENEMY():

	def __init__(self, idAgent):
		self.id = idAgent

	#Methods

	#Flight variables
	def getPosition(self, data):
		self.position = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z] #[deg, deg, m]
		return(self.position)

#def enemy_position_f(enemy_position):
        #e_x = enemy_position.pose.pose.position.x
        #e_y = enemy_position.pose.pose.position.y
        #e_z = enemy_position.pose.pose.position.z

def GNC_Node_Action():
	print("Intitialisation process has begun")
	global processOrderFlag

	#Initialise instance of UAV class and the associated Autopilot`
	uav = UAV(idAgent = 1)
        enemy = ENEMY(idAgent = 2)

	print("Begin Subsribers and Publishers")
	# Initialise node, anonymous necessary for centralised system
	rospy.init_node('GNC_Node_Action', anonymous=True)
	rospy.loginfo("Initialised Node")

	'''
	# Initialise publishing to TaskActions topic
	pubGlobal = rospy.Publisher("FriendInfo", AgentInfo, queue_size=10)
	pubLocal = rospy.Publisher('LocalInfo', AgentInfo, queue_size=10)
	rospy.loginfo("Initialised Publishing")


	# Initialise subscribing to TaskActions topic
	rospy.Subscriber("TaskActions", String, getOrders) #Every time a message is received, callback() is called
	rospy.loginfo("Initialised Subscribing TaskActions")
	'''

	# Initialise subscribing to Orders topic
	#rospy.Subscriber("Orders", OrderList, uav.getOrders) #Every time a message is received, callback() is called
	#rospy.loginfo("Initialised Subscribing Orders")


	# Initialise subscribing to GPS location topic
	rospy.Subscriber("/mavros/global_position/local", Odometry, uav.getPosition)
	rospy.loginfo("Initialised Subscribing MAVROS/global_position/local") #local instead of global

	#rospy.Subscriber("/iris/mavros/global_position/global", NavSatFix, uav.getPosition)
	#rospy.loginfo("Initialised Subscribing MAVROS/global_position/local") #local instead of global

	# Initialise subscribing to Heading topic
	rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, uav.getHeading)
	rospy.loginfo("Initialised Subscribing MAVROS/global_position/compass_hdg")

	# Initialise subscribing to IMU topic
	rospy.Subscriber("/mavros/imu/data", Imu, uav.getIMU)
	rospy.loginfo("Initialised Subscribing MAVROS/imu/data")

	#Initialise subscribing to BatteryState topic
	rospy.Subscriber("/mavros/battery", BatteryState, uav.getBatteryStatus)
	rospy.loginfo("Initialised Subscribing MAVROS/battery")

	#Initialise subscribing to State topic
	rospy.Subscriber("/mavros/state", State, uav.getState)
	rospy.loginfo("Initialised Subscribing MAVROS/state")

	#Initialise subscribing to WaypointReached topic
	rospy.Subscriber("/mavros/mission/reached", WaypointReached, waypointReached)
	rospy.loginfo("Initialised Subscribing MAVROS/mission/reached")

	#Initialise subscribing to WaypointReached topic
	rospy.Subscriber("/cam_p", Odometry, enemy.getPosition)
	rospy.loginfo("Initialised Subscribing /cam_p")

	import dronekit_sitl

	vehicle = connect('127.0.0.1:14560', wait_ready=True) #'udp:localhost:14551', wait_ready=True)  #127.0.0.1:14560

	#rospy.spin() #Keep the nodes active
	print("Begin Orders")

	vehicle.mode = VehicleMode("GUIDED")
        
        #uav.setFlightMode("GUIDED")

	rate = rospy.Rate(0.2) #Hz

	#uav.abortMission() #added

	#wait4connect() #added

	rate.sleep() #NECESSARY TO GIVE TIME TO GET THE ATTRIBUTES

	#uav.setHomeLocation(False, [0, 0, 0])

	uav.arm()

	#uav.setFlightMode(mode='GUIDED') #Changed from STABILIZE to GUIDED and uncommented


	uav.takeOff()

	rate.sleep()


	rate.sleep() #uncommented

	#waypoints = [[5, 5, 3], [5, -5, 3], [-5, -5, 5], [-5, 5, 5]] 

	waypoints = [[enemy.position[1],enemy.position[0],enemy.position[2]+1]]
        print(waypoints)

	
        #t = np.arange(20)
	#for k in t:
	    #waypoints.append([3*np.sin(k),k,5])



	uav.go_to_defending(waypoints,vehicle)

	#uav.setFlightMode("AUTO")

	#uav.land()

	rate.sleep()

	#uav.disarm()

	rate.sleep()

	print('END')


	'''
	# Initialisospy.spin() #Keep the nodes activee message
	agentMsg = AgentInfo()
	agentMsg.idAgent = uav.id
	rospy.loginfo("Initialised Message")
	'''




if __name__ == '__main__':
	try:
		GNC_Node_Action()
	except rospy.ROSInterruptException:
		pass
