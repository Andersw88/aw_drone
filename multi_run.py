#!/usr/bin/env python

import sys,os,subprocess
import time
from random import random
from pprint import pprint
import csv

class Runner():
	
	def __init__(self,numDrones,iterations,tdms,mpms):
		self.numDrones = numDrones
		self.tdms = tdms
		self.mpms = mpms
		self.start = time.time()
		self.iterations = iterations
		self.totalIterations = 0
		
		filePath = os.path.join(os.path.dirname(__file__),"aw_multi_solver_wrapper/src/maze3.pgm")
		with open(filePath, 'r') as content_file:
			maptext = content_file.read().split('\n')
			
		scale = 10
		self.mapSize = [ int(x) for x in maptext[2].split(' ') ]

		mapData = [ int(x != '0') for x in maptext[4:-1]]

		self.mapData =  [mapData[i:i+self.mapSize[0]] for i in range(0, len(mapData), self.mapSize[0])]
		self.starts,self.goals = self.genStartsAndGoals(numDrones)
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

	#def run(self,taskDeligatorMode,multiplannerMethod):
		
		#for i in range(0,self.iterations):
			#args = ['roslaunch', 'aw_hector_quadrotor', 'maze3-%s.launch'%(numDrones,),'rviz:=0','run_id:=%s'%(i,), 'tdm:=%s'%(taskDeligatorMode,),'mpm:=%s'%(multiplannerMethod,),'starts:=%s'%(self.starts[i],),'goals:=%s'%(self.goals[i],)]
			#subprocess.check_output(args)
			#self.totalIterations += 1
			#print 'Finnished %s iteration. Method:%s,%s. Total time spent:%s'%(self.totalIterations,taskDeligatorMode,multiplannerMethod,time.time()-self.start)
			

	def runAll(self):
		
		for i in range(0,self.iterations):
			self.logProblem(i)
			for tdm in self.tdms:
				for mpm in self.mpms:
					args = ['roslaunch', 'aw_hector_quadrotor', 'maze3-%s.launch'%(self.numDrones,),'rviz:=0','run_id:=%s'%(i,), 'tdm:=%s'%(tdm,),'mpm:=%s'%(mpm,),'starts:=%s'%(self.starts[i],),'goals:=%s'%(self.goals[i],)]
					subprocess.check_output(args)
					self.totalIterations += 1
					print 'Finnished iteration %s, total runs %s. Method:%s,%s. Total time spent:%s'%(i,self.totalIterations,tdm,mpm,time.time()-self.start)
	
	def logProblem(self,i):
		filePath = os.path.join(os.path.dirname(__file__),'problem_setups.csv')
		with open(filePath, 'a') as csvfile:
			resultWrite = csv.writer(csvfile, delimiter=',',
			quotechar='|', quoting=csv.QUOTE_MINIMAL)
			resultWrite.writerow([i,self.starts[i],self.goals[i]])

if __name__ == '__main__':
	
	runner = Runner(4,50,[0,1,2],['PP'])
	runner.runAll()
	runner = Runner(6,50,[0,1,2],['PP'])
	runner.runAll()
	runner = Runner(8,50,[0,1,2],['PP'])
	runner.runAll()
	runner = Runner(10,50,[0,1,2],['PP'])
	runner.runAll()
	
	print "Python script is now finnished"

