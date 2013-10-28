#!/usr/bin/env python

import sys,os,subprocess
import time
from random import random
from pprint import pprint
import csv
import sqlite3

class Runner():
	
	def __init__(self,numDrones,iterations,tdms,mpms,mapName):
		self.mapName = mapName
		self.numDrones = numDrones
		self.maxDrones = max(numDrones)
		self.tdms = tdms
		self.mpms = mpms
		self.start = time.time()
		self.iterations = iterations
		self.totalIterations = 0
		
		filePath = os.path.join(os.path.dirname(__file__),"aw_multi_solver_wrapper/src/%s.pgm"%mapName)
		with open(filePath, 'r') as content_file:
			maptext = content_file.read().split('\n')
			
			
		scale = 10
		self.mapSize = [ int(x) for x in maptext[2].split(' ') ]

		mapData = [ int(x != '0') for x in maptext[4:-1]]

		self.mapData =  [mapData[i:i+self.mapSize[0]] for i in range(0, len(mapData), self.mapSize[0])]
		self.starts,self.goals = self.genStartsAndGoals(self.maxDrones)
		#print self.goals

		
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

	def runAll(self):
		#pprint (self.numDrones)
		for i in range(0,self.iterations):
			run_id = self.logProblem(i)
			for droneCount in self.numDrones:
				for tdm in self.tdms:
					for mpm in self.mpms:
						args = ['roslaunch', 'aw_hector_quadrotor', '%s-%s.launch'%(self.mapName,droneCount,),'rviz:=1','run_id:=%s'%(run_id,), 'tdm:=%s'%(tdm,),'mpm:=%s'%(mpm,),'starts:=%s'%(self.starts[i][0:droneCount],),'goals:=%s'%(self.goals[i][0:droneCount],)]
						subprocess.check_output(args)
						self.totalIterations += 1
						print 'Finnished iteration %s, total runs %s. Method:%s,%s. Total time spent:%s'%(i,self.totalIterations,tdm,mpm,time.time()-self.start)
		
	def logProblem(self,i):
		filePath = os.path.join(os.path.dirname(__file__),'problem_setups.csv')
		with open(filePath, 'a') as csvfile:
			resultWrite = csv.writer(csvfile, delimiter=',',
			quotechar='|', quoting=csv.QUOTE_MINIMAL)
			resultWrite.writerow([i,self.starts[i],self.goals[i]])
			
		problemData = (', '.join([str(x) for x in self.starts[i]]),', '.join([str(x) for x in self.goals[i]]),self.mapName)
		#print problemData
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
	
	runner = Runner([4,6,8,10],50,[0,2,3],['PP'],'maze4')
	runner.runAll()
	runner = Runner([4,6,8,10],50,[0,2,3],['PP'],'maze3')
	runner.runAll()
	#runner = Runner(8,50,[0,1,2],['PP'])
	#runner.runAll()
	#runner = Runner(10,50,[0,1,2],['PP'])
	#runner.runAll()
	
	print "Python script is now finnished"

