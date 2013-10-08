#!/usr/bin/env python

import sys,os,subprocess
import time


class Runner():
	
	def __init__(self):
		self.start = time.time()
		self.totalIterations = 0
	def startNTimes(self,taskDeligatorMode,multiplannerMethod,n = 10):
		args = ['roslaunch', 'aw_hector_quadrotor', 'maze3.launch','rviz:=0', 'tdm:=%s'%(taskDeligatorMode,),'mpm:=%s'%(multiplannerMethod,)]
		for i in range(0,n):
			subprocess.check_output(args)
			self.totalIterations += 1
			print 'Finnished %s iteration. Method:%s,%s. Total time spent:%s'%(self.totalIterations,taskDeligatorMode,multiplannerMethod,time.time()-self.start)

if __name__ == '__main__':
	runner = Runner()
	
	runner.startNTimes(0,'PP',10)
	runner.startNTimes(0,'IIHP',10)
	runner.startNTimes(2,'PP',10)
	runner.startNTimes(2,'IIHP',10)
	print "Python script is now finnished"

