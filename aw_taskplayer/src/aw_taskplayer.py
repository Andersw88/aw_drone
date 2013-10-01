#!/usr/bin/env python
import roslib; roslib.load_manifest('aw_taskplayer')
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import String
import tf
from pprint import pprint
import os

#class Drone():
	#self.name
	#self.position
	#self.path = []
	

class TaskPlayer():
    
	def __init__(self):
		self.currentTimeStepIndex = 0
		
		dronePaths = ''
		filePath = os.path.join(os.path.dirname(__file__), 'paths.txt')
		with open(filePath, 'r') as content_file:
			dronePaths = content_file.read()
		#print tempPaths
		lineSeperatedPaths = dronePaths.strip(", \n").split("\n")
		commaSeperatedPaths = [i.strip(", ").split(",") for i in lineSeperatedPaths]
		spaceSeperatedPaths = [[j.strip(", ").split(" ")for j in i] for i in commaSeperatedPaths]
		
		self.timeSteps = list(set([int(x[0]) for sublist in[[[j[0]] for j in i] for i in spaceSeperatedPaths] for x in sublist]))
		self.timeSteps.sort()
		#print self.timeSteps
		dictsWithPositions = [{int(j[0]):[float(j[1])/100.0,float(j[2])/100.0] for j in i} for i in spaceSeperatedPaths]
		
		#pprint (lineSeperatedPaths)
		#pprint (commaSeperatedPaths)
		#pprint (spaceSeperatedPaths)
		#pprint (dictsWithPositions)
		droneNames = []
		droneNames = rospy.get_param('drones', droneNames)
		self.drones_ = [{} for _ in range(0,len(droneNames))]

		for i in range(0,len(droneNames)):
			self.drones_[i]['name'] = droneNames[i]

		
		
		for i,drone in enumerate(self.drones_):
			drone['path'] = dictsWithPositions[i]
			drone['position'] = [None,None]
			drone['nextPoseReached'] = True
			drone['referenceSet'] = False
			
			drone['currentGoal'] = drone['path'][0]
			drone['commmadPub'] = rospy.Publisher(drone['name'] + '/tum_ardrone/com', String)
			#drone['pathPub'] = rospy.Publisher(drone['name'] + '/nav_msgs/Path', String)
			#drone['commmadPub'] = rospy.Publisher(drone['name'] + '/goal', Point)
			drone['poseSub'] = rospy.Subscriber(drone['name'] + '/ground_truth/state', Odometry, self.poseSubscribers_rospy, drone)
			
			
			

		
		pprint(self.drones_)
		#print 'c goto(',self.drones_[0]['currentGoal'][0],self.drones_[0]['currentGoal'][1],')'
		#self.update()
		
	def update(self):
		#rospy.loginfo("update called")

		if(all(i['nextPoseReached'] for i in self.drones_)):
			if(self.currentTimeStepIndex >= len(self.timeSteps)):
				rospy.loginfo("TaskPlayer is now finnished")
				return
			self.currentTimeStepIndex += 1
			
			for drone in self.drones_:
				if(self.timeSteps[self.currentTimeStepIndex] in drone['path']):
					drone['nextPoseReached'] = False
					drone['currentGoal'] = drone['path'][self.timeSteps[self.currentTimeStepIndex]]
					self.publishCommand(drone)
					
	def poseSubscribers_rospy(self, msg, drone):
		#rospy.loginfo("poseSubscribers_rospy In callback")
		
		
		drone['position'][0] = msg.pose.pose.position.x
		drone['position'][1] = msg.pose.pose.position.y
		
		#if(drone['referenceSet'] == False):
			#drone['commmadPub'].publish(String("c setReference %f %f 1.0 1.57"%(drone['position'][0],drone['position'][1])) )
			#drone['referenceSet'] = True
			
		if(((drone['currentGoal'][0] - drone['position'][0])**2 + 
			(drone['currentGoal'][1] - drone['position'][1])**2)**0.5  < 0.5 and not drone['nextPoseReached']):
			rospy.loginfo(drone['name'] + " reached step %f %f,timestep: %i" %(drone['currentGoal'][0],drone['currentGoal'][1],self.timeSteps[self.currentTimeStepIndex]))
			drone['nextPoseReached'] = True
		
	def publishCommand(self,drone):
		#if(drone['position'][0] != None):
		s = 'c goto %s %s 0.7 0.0' %(drone['currentGoal'][0],drone['currentGoal'][1],)
		rospy.loginfo(s)
		drone['commmadPub'].publish(String(s))
		#rospy.loginfo("x:%s,y:%s,z:%s"%(drone['currentGoal'][0],drone['currentGoal'][1],1.0))
		#drone['commmadPub'].publish(Point(drone['currentGoal'][0],drone['currentGoal'][1],0.0))

	def delayedInit(self):
		for drone in self.drones_:
			drone['commmadPub'].publish(String("c autoTakeover 500 800"))
			drone['commmadPub'].publish(String("c start"))
			drone['commmadPub'].publish(String("c setStayWithinDist 0.5"))
			drone['commmadPub'].publish(String("c setInitialReachDist 0.2"))
			drone['commmadPub'].publish(String("c setReference 0 0 0 0"))
			drone['commmadPub'].publish(String("c setMaxControl 5.0"))
			drone['commmadPub'].publish(String("c setStayTime 0.0"))
			
			drone['commmadPub'].publish(String("autoTakeover 500 800"))
			#print ""
		
if __name__ == '__main__':
	rospy.init_node('aw_taskplayer', anonymous=True)

	taskPlayer = TaskPlayer()
	rospy.sleep(10.0) #Needed to let publisher initialize
	taskPlayer.delayedInit()
	while not rospy.is_shutdown():
		taskPlayer.update()
		
		rospy.sleep(0.2)
        
