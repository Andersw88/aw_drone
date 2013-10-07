#!/usr/bin/python
import rospy
import sys,os
from pprint import pprint
from time import sleep
from random import random


if __name__ == '__main__':
		rospy.init_node('aw_ascii_pgm_to_random_goals', anonymous=True)	
	
		numDrones = len(rospy.get_param('drones', []))
		map_name = ''
		
		while(map_name == ''):
			map_name = rospy.get_param('/aw/map_name', map_name)
			#print map_name
			rospy.loginfo("Waiting for map_name")
			sleep(1)

		
		filePath = os.path.join(os.path.dirname(__file__),map_name+".pgm")
		with open(filePath, 'r') as content_file:
			maptext = content_file.read().split('\n')
			
		scale = 10
		
		#if(len(sys.argv) == 3):
			#scale = int(sys.argv[2])
		
		mapSize = [ int(x) for x in maptext[2].split(' ') ]
		#mapData = [ bool(x) for x in maptext[4:-1]]
		mapData = [ int(x != '0') for x in maptext[4:-1]]
		
		#print mapData
		
		mapData =  [mapData[i:i+mapSize[0]] for i in range(0, len(mapData), mapSize[0])]
		#for i in mapData:
			#print i
		#pprint(mapData)
		#numGoalsSet = 0
		goals = set()
		while(len(goals) < numDrones):
			x = int(random()*(mapSize[0]))
			y = int(random()*(mapSize[1]))
			#print x,y,mapSize[1]-y-1
			if(mapData[y][x] != 0):
				goals.add(tuple((x+0.5,mapSize[1]-y-0.5)))
		#pprint(list(goals))
		#pprint([[goal[0],goal[1]] for goal in goals])
		rospy.set_param('aw/goals', [[goal[0],goal[1]] for goal in goals])
		#pprint(goals)
			
		#for i in range(0,mapSize[0]):
			#for j in range(0,mapSize[1]):
				#if(mapData[i+j*mapSize[0]] == "0"):
					#x = i*scale
					#y = j*scale
					