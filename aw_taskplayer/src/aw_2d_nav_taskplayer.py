#!/usr/bin/env python
import roslib; roslib.load_manifest('aw_taskplayer')
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion


from std_msgs.msg import String
from std_msgs.msg import Header
import tf
from pprint import pprint
from aw_multi_solver_wrapper.srv import *
from std_srvs.srv import *

import os,subprocess
import time
import csv

import sqlite3




class TaskPlayer():
    
	def __init__(self):
		
		self.currentTimeStepIndex = 0
		
		droneNames = []
		droneNames = rospy.get_param('drones', droneNames)
		#droneNames = rospy.get_param('drones', droneNames)
		#timeoutTimer = time.time()
		#while(len(droneNames) == 0):
			#rospy.info("Waiting for param 'drones'")
			#droneNames = rospy.get_param('drones', droneNames)
			#time.sleep(1)
			
		self.goalTolerance_ = rospy.get_param('aw/goal_tolerance', droneNames)
		multiPlannerMapScale = float(rospy.get_param('aw/multiplanner/map_scale', 10.0))
		self.drones_ = [{} for _ in range(0,len(droneNames))]

		for i in range(0,len(droneNames)):
			self.drones_[i]['name'] = droneNames[i]
		for i,drone in enumerate(self.drones_):
			drone['commmadPub'] = rospy.Publisher(drone['name'] + '/goal', Point)
			drone['pathPub'] = rospy.Publisher(drone['name'] + '/nav_msgs/Path', Path)
			
		self.success = False
		try:
			rospy.wait_for_service('getMultiPlan', timeout = 600)
		except rospy.ROSException:
			self.start_time = rospy.get_rostime()
			self.success = False
			self.planTime = 0
			self.writeResult()
			rospy.signal_shutdown("Timed out")
			return
			
		startPlanTime = time.time()
		dronePaths = self.getPlan()
		self.planTime = time.time()-startPlanTime
		self.success = True;
		#print ":",dronePaths[0:2], ": <-------------------------------------------------------" 
		if(dronePaths == '' or dronePaths[0:2] == "No"):
			self.success = False;
			self.writeResult()
			rospy.signal_shutdown("Failed to find solution")
			return
			
		pprint(dronePaths)
		lineSeperatedPaths = dronePaths.strip(", \n").split("\n")
		commaSeperatedPaths = [i.strip(", ").split(",") for i in lineSeperatedPaths]
		spaceSeperatedPaths = [[j.strip(", ").split(" ")for j in i] for i in commaSeperatedPaths]
		
		self.timeSteps = list(set([int(x[0]) for sublist in[[[j[0]] for j in i] for i in spaceSeperatedPaths] for x in sublist]))
		self.timeSteps.sort()
		#print self.timeSteps
		dictsWithPositions = [{int(j[0]):[float(j[1])/multiPlannerMapScale,float(j[2])/multiPlannerMapScale] for j in i} for i in spaceSeperatedPaths]
		
		#pprint (lineSeperatedPaths)
		#pprint (commaSeperatedPaths)
		#pprint (spaceSeperatedPaths)
		#pprint (dictsWithPositions)

		
		self.isLive = rospy.get_param('aw/live', False)

		for i,drone in enumerate(self.drones_):
			drone['path'] = dictsWithPositions[i]
			drone['position'] = [None,None]
			drone['nextPoseReached'] = True
			drone['referenceSet'] = False
			drone['currentGoal'] = drone['path'][0]
			drone['poseSub'] = rospy.Subscriber(drone['name'] + '/ground_truth/state', Odometry, self.poseSubscribers_rospy, drone)
			
			
			process = subprocess.Popen(["rosnode kill %s/move_base"%(drone['name'],)],stdout=subprocess.PIPE, stdin=subprocess.PIPE, shell=True)
			stdout, stderr = process.communicate()
			#drone['commmadPub'] = rospy.Publisher(drone['name'] + '/tum_ardrone/com', String)

			
			
		print "Total distance = ",self.calculateTotalTravelDistance();
		if(self.isLive):
			rospy.loginfo("aw_2d_nav_taskplayer: Executing plan for live drones")
		else:
			rospy.loginfo("aw_2d_nav_taskplayer: Executing plan for simulated drones")
			
			
		self.publishDronePaths()
		
		#pprint(self.drones_)
		#print 'c goto(',self.drones_[0]['currentGoal'][0],self.drones_[0]['currentGoal'][1],')'
		#self.update()
		
	def getPlan(self):
		try:
			multiPlannerClient = rospy.ServiceProxy('getMultiPlan', MultiPlanner)
			resp1 = multiPlannerClient()
			return resp1.plans
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return ''
	
	def writeResult(self):
		filePath = os.path.join(os.path.dirname(__file__),'../../results.csv')
		planExecutionTime = None
		totalTravelDistance = None
		if(self.success):
			planExecutionTime = (rospy.get_rostime() - self.start_time).secs
			totalTravelDistance = self.calculateTotalTravelDistance()
		
		method = rospy.get_param('aw/multiplanner/method', '')
		tdm = rospy.get_param('aw/goal_deligator_mode',0)
		run_id = rospy.get_param('aw/run_id',-1)
		mapName = rospy.get_param('aw/map_name','')
		
		
		print filePath
		
		runData = (run_id,len(self.drones_),method,tdm,"%.2f"%(self.planTime,),self.success,planExecutionTime, totalTravelDistance,mapName)
		with open(filePath, 'a') as csvfile:
			resultWrite = csv.writer(csvfile, delimiter=',',
									quotechar='|', quoting=csv.QUOTE_MINIMAL)
			resultWrite.writerow(runData)
		
		dbPath = os.path.join(os.path.dirname(__file__),'../../results.sqlite')
		conn = sqlite3.connect(dbPath)
		c = conn.cursor()
		c.execute("""INSERT INTO runs 
					(problem_id, num_drones, mpm,tdm, plan_time, plan_success,plan_execution_time,total_travel_distance,map_name) 
					VALUES (?,?,?,?,?,?,?,?,?)""",runData)
		conn.commit()
		c.close()
	
	
	def update(self):
		#rospy.loginfo("update called")
		if(self.currentTimeStepIndex == 0):
			self.start_time = rospy.get_rostime()
			print "Start time:", self.start_time.secs
			
		if((rospy.get_rostime() - self.start_time).secs >= 600):
			self.writeResult()
			rospy.signal_shutdown("Timed out")
			return

		if(all(i['nextPoseReached'] for i in self.drones_)):
			self.currentTimeStepIndex += 1
			if(self.currentTimeStepIndex >= len(self.timeSteps)):
				self.writeResult()
				rospy.loginfo("TaskPlayer is now finnished, time since start %s" %((rospy.get_rostime() - self.start_time).secs,))
				rospy.signal_shutdown("Finnished")
				return
			
			for drone in self.drones_:
				if(self.timeSteps[self.currentTimeStepIndex] in drone['path']):
					drone['nextPoseReached'] = False
					drone['currentGoal'] = drone['path'][self.timeSteps[self.currentTimeStepIndex]]
					self.publishCommand(drone)
					
	def poseSubscribers_rospy(self, msg, drone):
		drone['position'][0] = msg.pose.pose.position.x
		drone['position'][1] = msg.pose.pose.position.y
		
		if(((drone['currentGoal'][0] - drone['position'][0])**2 + 
			(drone['currentGoal'][1] - drone['position'][1])**2)**0.5  < self.goalTolerance_ and not drone['nextPoseReached']):
			rospy.logdebug(drone['name'] + " reached step %f %f,timestep: %i" %(drone['currentGoal'][0],drone['currentGoal'][1],self.timeSteps[self.currentTimeStepIndex]))

			drone['nextPoseReached'] = True
	
	def calculateTotalTravelDistance(self):
		
		distSum = 0
		for drone in self.drones_:
			for i in range(0,len(self.timeSteps)-1):
				if(self.timeSteps[i] in drone['path'] and self.timeSteps[i+1] in drone['path']):
					distSum += ((drone['path'][self.timeSteps[i+1]][0] - drone['path'][self.timeSteps[i]][0])**2 + (drone['path'][self.timeSteps[i+1]][1] - drone['path'][self.timeSteps[i]][1])**2)**0.5
		return distSum
				
			
			
	def publishDronePaths(self):
		q = Quaternion(w=1)
		for drone in self.drones_:
			path =  Path()
			path.header.frame_id = "map"
			path.poses = [ PoseStamped(header = Header(seq=i,frame_id="map"),pose=Pose(position=Point(x=drone['path'][key][0],y=drone['path'][key][1]),orientation=q)) for i,key in enumerate(sorted(drone['path']))]
			drone['pathPub'].publish(path)
			#subprocess.Popen(list(args), stdout=subprocess.PIPE, stderr=subprocess.PIPE)

		
		
		
	def publishCommand(self,drone):
		#if(drone['position'][0] != None):
		#s = 'c goto %s %s 0.7 0.0' %(drone['currentGoal'][0],drone['currentGoal'][1],)
		#rospy.loginfo(s)
		#drone['commmadPub'].publish(String(s))
		#rospy.loginfo("x:%s,y:%s,z:%s"%(drone['currentGoal'][0],drone['currentGoal'][1],1.0))
		drone['commmadPub'].publish(Point(drone['currentGoal'][0],drone['currentGoal'][1],0.0))

	#def delayedInit(self):
		#for drone in self.drones_:
			#drone['commmadPub'].publish(String("c autoTakeover 500 800"))
			#drone['commmadPub'].publish(String("c start"))
			#drone['commmadPub'].publish(String("c setStayWithinDist 0.5"))
			#drone['commmadPub'].publish(String("c setInitialReachDist 0.2"))
			#drone['commmadPub'].publish(String("c setReference 0 0 0 0"))
			#drone['commmadPub'].publish(String("c setMaxControl 5.0"))
			#drone['commmadPub'].publish(String("c setStayTime 0.0"))
			
			#drone['commmadPub'].publish(String("autoTakeover 500 800"))
			##print ""
		
if __name__ == '__main__':
	rospy.init_node('aw_taskplayer', anonymous=True)

	taskPlayer = TaskPlayer()
	#rospy.sleep(10.0) #Needed to let publisher initialize
	#taskPlayer.delayedInit()

	while not rospy.is_shutdown():
		rospy.sleep(0.01)
		taskPlayer.update()
		#taskPlayer.publishDronePaths()
		
        
