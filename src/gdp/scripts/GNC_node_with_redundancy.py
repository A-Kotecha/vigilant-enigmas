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
		self.position = [data.latitude, data.longitude, data.altitude] #[deg, deg, m]

	def getHeading(self, data):
		self.heading = data #deg

	def getIMU(self, data):
		#self.IMU_acceleration = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z #m/s2
		self.IMU_rate = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z] #rad/s

	# def getVelocity(self, data): #Global or Local?
	# 	self.velocity_lin = ??
	# 	self.velocity_ang = ??

	def getBatteryStatus(self, data):
		self.battery = data.percentage #percentage
		print("Checking Battery")

	def getState (self, data):
		self.state = [data.connected, data.armed, data.guided, data.mode, data.system_status]
		print(self.state)
		print("Getting State")

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
		'''
		while (self.state[3] != mode):
			print("\nSetting Mode")
			rospy.loginfo("Request change flight mode to %s" %mode)
			rospy.wait_for_service('/mavros/set_mode')
			try:
				setModeRequest = rospy.ServiceProxy('/mavros/set_mode', SetMode)
				setModeResponse = setModeRequest(custom_mode = "GUIDED")
				rospy.loginfo(setModeResponse)
			except rospy.ServiceException as e:
				rospy.loginfo("Request change flight mode failed: %s" %e)
		#send Message
		if self.state[3] == mode:
			rospy.loginfo("SUCCESS: Set Flight Mode:GUIDED ")
		else:
			rospy.loginfo("FAILURE: Set Flight Mode:GUIDED ")
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
				print("Bring up Camp 18 again. Tighten on the camp here.")
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
		'''
		if self.state[1] != True:
			self.arm()


		global takeOffFlag
		
		takeOffFlag = False
		groundAlt = self.position[2]

		#self.setFlightMode(mode = 'AUTO.TAKEOFF')	
		self.setFlightMode(mode = 'AUTO.TAKEOFF') #CHANGE

		while not abs(self.position[2]-groundAlt) < 0.30: # meter
			pass
		takeOffFlag = True

		#Send messageosition[0
		rospy.loginfo("SUCCESS: TakeOff")
		#Add timeout??


		#while takeOffFlag == False:
		#	if relative altitude reached:
		#		takeOffFlag = True

		'''
		#Manual TakeOff
		self.setFlightMode('GUIDED')
		print("TKOFF")
		rospy.loginfo("Request TakeOff")
		rospy.wait_for_service('/mavros/cmd/takeoff')
		alt = 10 # Added just to make run, no idea where it is from. alt in takeOffResponse where here before
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
		
		

	def waypoint(self, coordinates): #coordinates = [lat, long, alt]

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
		wp.param2 = 0 #Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
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
				print("Waypoint Pushed")
			except rospy.ServiceException as e:
				rospy.loginfo("Request new mission plan failed: %s" %e)
			counter += 1

		#Message?
        	
		#if not in AUTO.MISSION, set it into this mode
		if self.state[3] != 'AUTO':
			self.setFlightMode(mode = 'AUTO')
			print("Sir, a jelly doughnut")

		while waypointFlag == False: #Is True when the WP is reached
			#print("This is the waypoint %s" %waypointFlag)
			pass

		#self.setFlightMode(mode = 'AUTO.LOITER')
        

		

	def prelease(self):
		pass

	def repaint (self):
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

	##ORDER PROCESSING



	'''
	
	commandType = 1, 2, 3, 4, 5, 6, 7, 8
	With:
		1: TAKEOFF(alt)
		2: WAYPOINT(GPS coord)
		3: PRELEASE
		4: REPAINT
		5: LAND
		6: ABORTMISSION
		7: ARM
		8: DISARM
		
	'''

	def getOrders(self, data):

		print(data)

		

		global ordersStack

		#Check whether the order is ABORT

		if data.order[0].order_name == 6:
			if self.id == data.order[0].idAgent:
				rospy.loginfo("%s: ABORTMISSION" % rospy.get_time())
				self.abortMission()

		#if not, add the orders to the stack
		else:
			ordersStack += data.order


	def processOrder(self):

		#Lock order processing
		processOrderFlag = False

		# Unstack an order
		if len(ordersStack) != 0: 
			order = ordersStack.pop(0)
			print(order)
		else:
			processOrderFlag = True
			print("Process Order")
			return

		# Process the order

		#Check agent ID
		

		if order.idAgent != self.id:
			return

		else:

			if order.order_name == 1:
				rospy.loginfo("%s: TAKEOFF" % rospy.get_time())
				self.takeOff()
				processOrderFlag = True	
				print("TK")
			elif order.order_name == 2:
				rospy.loginfo("%s: WAYPOINT" % rospy.get_time())
				self.waypoint(order.order_info)
				processOrderFlag = True
				print("WP")

			elif order.order_name == 3:
				rospy.loginfo("%s: PRELEASE" % rospy.get_time())
				self.prelease()
				processOrderFlag = True
				print("PL")

			elif order.order_name == 4:
				rospy.loginfo("%s:REPAINT" % rospy.get_time())
				self.repaint()
				processOrderFlag = True
				print("PR")

			elif order.order_name == 5:
				rospy.loginfo("%s: LAND" % rospy.get_time())
				self.land()
				processOrderFlag = True
				print("land")
			elif order.order_name == 7:
				rospy.loginfo("%s: ARM" % rospy.get_time())
				self.arm()
				processOrderFlag = True
				print("A")
			elif order.order_name == 8:
				rospy.loginfo("%s: DISARM" % rospy.get_time())
				self.disarm()
				processOrderFlag = True
				print("D")
			else:
				#send ORDER not understood to Task Allocation
				processOrderFlag = True
				print("POF")
				pass

	##End of ORDER PROCESSING

	
##End class


#Other parameters related to the mission


def waypointReached(data): #returns the index number of the reached waypoint
	#We assume that there is only one waypoint in the mission table at the same time
	global waypointFlag
	waypointFlag = True
	
'''
# this exists as rospy.is_shutdown() ?
def wait4connect():
	rospy.info("Waiting for FCU connection")
	while (not rospy.is_shutdown()) and (not state.connected):
		rospy.spin(1)
		rospy.sleep(0.01)
	if state.connected:
		rospy.info("Connected to FCU")
		return 0
	else:
		rospy.info("Error connecting to drone")
		return -1	
'''
##INITIALISATION

def initialisation():
	pass
 
def GNC_Node_Action():
	print("Intitialisation process has begun")
	global processOrderFlag

	#Initialise instance of UAV class and the associated Autopilot`
	uav = UAV(idAgent = 1) #TBC who is giving the ID? GNC or Task Allocation?

	
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
	rospy.Subscriber("Orders", OrderList, uav.getOrders) #Every time a message is received, callback() is called
	rospy.loginfo("Initialised Subscribing Orders")
	
	
	# Initialise subscribing to GPS location topic
	rospy.Subscriber("/mavros/global_position/global", NavSatFix, uav.getPosition) 
	rospy.loginfo("Initialised Subscribing MAVROS/global_position/global")
	
	# Initialise subscribing to Heading topic
	rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, uav.getHeading )
	rospy.loginfo("Initialised Subscribing MAVROS/global_position/compass_hdg")

	# Initialise subscribing to IMU topic
	rospy.Subscriber("/mavros/imu/data", Imu, uav.getIMU) 
	rospy.loginfo("Initialised Subscribing MAVROS/imu/data")

	#Initialise subscribing to BatteryState topic
	rospy.Subscriber("/mavros/battery", BatteryState, uav.getBatteryStatus) 
	rospy.loginfo("Initialised Subscribing MAVROS/battery")
	#print("Arshavin")

	#Initialise subscribing to State topic
	rospy.Subscriber("/mavros/state", State, uav.getState) 
	rospy.loginfo("Initialised Su: Falsebscribing MAVROS/state")
	#print("Arteta")

	#Initialise subscribing to WaypointReached topic
	rospy.Subscriber("mavros/mission/reached", WaypointReached, waypointReached)
	rospy.loginfo("Initialised Subscribing MAVROS/mission/reached")
	#print("Bergkamp")

	#rospy.spin() #Keep the nodes active
	print("Begin Orders")

	rate = rospy.Rate(0.2) #Hz
    
	#wait4connect() #added
	
	rate.sleep() #NECESSARY TO GIVE TIME TO GET THE ATTRIBUTES
	
	uav.arm()

	uav.setFlightMode(mode='GUIDED') #Changed from STABILIZE to GUIDED and uncommented

	uav.setHomeLocation(True, [52.070666384, -0.62333084, 170]) # original
	#uav.setHomeLocation(False, [0, 0, 0])
	print("hello")
    
	uav.takeOff()

	rate.sleep()
	uav.setFlightMode(mode='AUTO')

	uav.waypoint([20, 20,20])

	rate.sleep() #uncommented
    
	print("Landing")
    
	uav.land()

	rate.sleep()

	uav.disarm()

	rate.sleep()

	print('END')
	'''
	'''

	'''
	# Initialisospy.spin() #Keep the nodes activee message
	agentMsg = AgentInfo()
	agentMsg.idAgent = uav.id
	rospy.loginfo("Initialised Message")

	'''
	
	while not rospy.is_shutdown(): # Main loop

		'''
			
		# Publish data to global & local topics
		agentMsg.status = 'IDLE'
		agentMsg.position = uav.position
		agentMsg.heading = uav.heading #ADD Heading to AgentInfo Class
		agentMsg.battery = uav.battery
		agentMsg.timestamp = rospy.get_time()
		
		pubLocal.publish(agentMsg)
		pubGlobal.publish(agentMsg)
	while not rospy.is_shutdown(): #Main Loop
		'''
		print("processOrderFlag: %s" %processOrderFlag)
		print("SHUTDOWN THING")

		if processOrderFlag == True:
			print("Shutdown = True?")
			uav.processOrder()
		
		rate.sleep()
		



if __name__ == '__main__':
    try:
        GNC_Node_Action()
    except rospy.ROSInterruptException:
        pass
		
