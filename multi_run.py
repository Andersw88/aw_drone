#!/usr/bin/env python

import sys,os,subprocess
import time
from random import random,shuffle
from pprint import pprint
import csv
import sqlite3

class Runner():
	
	def __init__(self,numDrones,iterations,tdms,mpms,mapName,start = None):
		self.mapName = mapName
		self.numDrones = numDrones
		self.maxDrones = max(numDrones)
		self.tdms = tdms
		self.mpms = mpms
		self.startTime = time.time()
		self.iterations = iterations
		self.totalIterations = 0
		#self.starts = starts
		
		filePath = os.path.join(os.path.dirname(__file__),"aw_multi_solver_wrapper/src/%s.pgm"%mapName)
		with open(filePath, 'r') as content_file:
			maptext = content_file.read().split('\n')
			
			
		scale = 10
		self.mapSize = [ int(x) for x in maptext[2].split(' ') ]

		mapData = [ int(x != '0') for x in maptext[4:-1]]

		self.mapData =  [mapData[i:i+self.mapSize[0]] for i in range(0, len(mapData), self.mapSize[0])]
		if(not start):
			self.starts,self.goals = self.genStartsAndGoals(self.maxDrones)
		else:
			self.starts =  [start for i in range(iterations)]
			self.goals = self.genGoals(self.maxDrones)

		#pprint (self.starts)
		#print self.goals
		#print self.starts

		
	def genStartsAndGoals(self,numDrones):
		startsList = []
		goalsList = []
		for i in range(0,self.iterations):
			goals = set()
			while(len(goals) < numDrones):
				x = int(random()*(self.mapSize[0]))
				y = int(random()*(self.mapSize[1]))

				if(self.mapData[y][x] != 0):
					goals.add(tuple((x+0.5,self.mapSize[1]-y-0.5)))
			goalsList.append([[goal[0],goal[1]] for goal in goals])
			
			starts = set()
			while(len(starts) < numDrones):
				x = int(random()*(self.mapSize[0]))
				y = int(random()*(self.mapSize[1]))
				start = tuple((x+0.5,self.mapSize[1]-y-0.5))
				if(self.mapData[y][x] != 0 and not(start in goals)):
					starts.add(start)
			startsList.append([[start[0],start[1]] for start in starts])
		return startsList,goalsList
		
	def genGoals(self,numDrones):
		goalsList = []
		startsSet = set((start[0][0],start[0][1]) for start in self.starts)
		#print startsSet
		for i in range(0,self.iterations):
			goals = set()
			while(len(goals) < numDrones):
				x = int(random()*(self.mapSize[0]))
				y = int(random()*(self.mapSize[1]))
				goalT = tuple((x+0.5,self.mapSize[1]-y-0.5))
				if(self.mapData[y][x] != 0 and not(goalT in startsSet)):
					goals.add(goalT)
			goalsList.append([[goal[0],goal[1]] for goal in goals])
		return goalsList

	def runAll(self):
		#pprint (self.numDrones)
		for i in range(0,self.iterations):
			run_id = self.logProblem(i)
			for droneCount in self.numDrones:
				for tdm in self.tdms:
					for mpm in self.mpms:
						args = ['roslaunch', 'aw_hector_quadrotor', 'sim%s.launch'%(droneCount,),'map_name:=%s'%self.mapName,'rviz:=1','run_id:=%s'%(run_id,), 'tdm:=%s'%(tdm,),'mpm:=%s'%(mpm,),'starts:=%s'%self.starts[i][0:droneCount],'goals:=%s'%(self.goals[i][0:droneCount],)]
						print ' '.join(args)
						subprocess.check_output(args)
						self.totalIterations += 1
						print 'Finnished iteration %s, total runs %s. Method:%s,%s. Total time spent:%s'%(i,self.totalIterations,tdm,mpm,time.time()-self.startTime)
						
						
	def logProblem(self,i):
		filePath = os.path.join(os.path.dirname(__file__),'problem_setups.csv')
		with open(filePath, 'a') as csvfile:
			resultWrite = csv.writer(csvfile, delimiter=',',
			quotechar='|', quoting=csv.QUOTE_MINIMAL)
			resultWrite.writerow([i,self.starts[i],self.goals[i]])
		
		problemData = (', '.join([str(x) for x in self.starts[i]]),', '.join([str(x) for x in self.goals[i]]),self.mapName)
		conn = sqlite3.connect('results.sqlite')
		c = conn.cursor()
		c.execute("INSERT INTO problem_setups (starts,goals,map_name) VALUES (?,?,?)",problemData)
		conn.commit()
		lastId = c.lastrowid
		c.close()
		return lastId
		
def createSQLiteDB():
	conn = sqlite3.connect('results.sqlite')
	c = conn.cursor()
	try:
		c.execute('''CREATE TABLE problem_setups
		(id INTEGER PRIMARY KEY AUTOINCREMENT, starts text,goals text,map_name text)''')
	except sqlite3.OperationalError as e:
		print e
	try:
		c.execute('''CREATE TABLE runs
		(id INTEGER PRIMARY KEY AUTOINCREMENT,problem_id integer, num_drones integer, mpm text,tdm text, plan_time real, plan_success boolean,plan_execution_time real,total_travel_distance real,map_name text,
		FOREIGN KEY(problem_id) REFERENCES problem_setups(id))''')
	except sqlite3.OperationalError as e:
		print e
	conn.commit()
	c.close()

if __name__ == '__main__':
	createSQLiteDB()
	
	#runner = Runner([4,6,8,10],2,[0,2,3],['PP'],'maze5',start= [[0.5, 0.5], [2.5, 0.5], [4.5, 0.5], [0.5, 2.5], [2.5, 2.5], [4.5, 2.5],[3.5, 3.5], [4.5, 4.5], [2.5, 4.5], [0.5, 4.5]])
	#runner.runAll()
	
	#First is the number of drones (can be 4,6,8 or 10)
	#Second is number of runs with the same setup, but different initial conditions.
	#Third are the task deligation modes. 0=Hungarian, 2=Hungarian with threshold, 3=old greedyfirst, 4 new greedyfirstv2.
	#Forth is the global planner planning mode 'PP' or 'IIHP', more?
	#Fifth is the map name. (maze3, maze4, or liuB2)
	
	runner = Runner([10],10,[0],['PP'],'liuB2')
	#runner = Runner([10],10,[0],['PP',IIHP],'liuB2')
	runner.runAll()


	
	print "Python script is now finnished"

