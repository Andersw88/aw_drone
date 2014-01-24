#!/usr/bin/python
import shlex, subprocess
#import roslib; roslib.load_manifest('aw_taskplayer')
#import roslib; roslib.load_manifest('aw_taskplayer')
import rospy
import sys,os
import json
from pprint import pprint
from aw_multi_solver_wrapper.srv import *
from aw_task_deligator.srv import *

#def wrapper(args):
    #process = subprocess.Popen(list(args), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    #stdout, stderr = process.communicate()
    #if(stdrr != ''):
		#print stderr
    #return stdout, stderr
    
class SolverWrapper():

	#def __init__(self,jsonObjectives):
		#dirPath = os.path.join(os.path.dirname(__file__))
		#self.args = ['java', '-jar', '%s/uber-solver.jar' %(dirPath), '-json',jsonObjectives]
		##self.args = ['java', '-jar', 'uber-solver.jar', '-json',jsonObjectives]
		##self.jsonObjectives = jsonObjectives
		
	def __init__(self,args):
		dirPath = os.path.join(os.path.dirname(__file__))
		self.args = ['java', '-jar', '%s/solver.jar' %(dirPath)]
		self.args.extend(args)
		#self.args = ['java', '-jar', 'uber-solver.jar', '-json',jsonObjectives]
		#self.jsonObjectives = jsonObjectives
		
		
	def callSolver(self):
		#args = [self.args,self.jsonObjectives]
		args = self.args
		#dirPath = os.path.join(os.path.dirname(__file__))
		process = subprocess.Popen(list(args), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		stdout, stderr = process.communicate()
		if(stderr):
			rospy.logerr("From java: " + stderr)
		return stdout
	
    
def callGetObjectives():
	
	try:
		objectivesClient = rospy.ServiceProxy('getObjectives', getObjectives)
		resp1 = objectivesClient()
		return resp1.objectives
	except rospy.ServiceException, e:
		rospy.logerr("aw_multi_solver_wrapper: getObjectives: Service call failed: %s"%e)
    
def multiPlannerService(req):
	
	objectives = callGetObjectives()
	
	map_name = rospy.get_param('aw/map_name', False)
	multiplanner_vis = rospy.get_param('aw/multiplanner/vis', True)
	multiplanner_map_scale = rospy.get_param('aw/multiplanner/map_scale', 10)
	grid_step = rospy.get_param('aw/multiplanner/grid_step', 5)
	drone_rad = rospy.get_param('aw/drone_rad', 0.5)
	maxtime = rospy.get_param('aw/multiplanner/maxtime', 400)
	timeout = rospy.get_param('aw/multiplanner/timeout', 20000)
	method = rospy.get_param('aw/multiplanner/method', 'PP')
	
	if(not(map_name)):
		rospy.logerr('aw_multi_solver_wrapper: param "aw/map_name" not set')
	
	#jsonObjectives = json.dumps({'method': method,
								#'problemfile': '%s/problems/%s.xml' %(os.path.join(os.path.dirname(__file__)),map_name,),
								#'timeout': timeout,
								#'showvis': multiplanner_vis,
								#'maxtime': maxtime,
								#'gridStep': grid_step,
								#'mission':[ {'startX':int(round(objectives.starts[i].x*multiplanner_map_scale/(5))*(5)),'startY':int(round(objectives.starts[i].y*multiplanner_map_scale/(5))*(5)),
											  #'targetX':int(objectives.targets[i].x*multiplanner_map_scale),'targetY':int(objectives.targets[i].y*multiplanner_map_scale),
											  #'rad':drone_rad*multiplanner_map_scale,
											  #'maxSpeed':1} for i in range(len(objectives.starts))]
											  
	args = ['-method',method,
			'-problemfile','%s/problems/%s.xml' %(os.path.join(os.path.dirname(__file__)),map_name,),
			'-timeout','%s' %timeout,
			#'-showvis','%s' %multiplanner_vis,
			'-maxtime', '%s' %maxtime,
			'-gridstep', '%s' %grid_step,
			'-mission', '%s' %(formatMissionString(objectives,drone_rad,multiplanner_map_scale)),
			'-k', '5'
			]
	if(multiplanner_vis):
		args.extend('-showvis')
								
								
								#'mission':[{'startsX':[int(start.x) for start in objectives.starts],
											##'startsY':[int(start.y) for start in objectives.starts],
											##'targetsX':[int(target.x) for target in objectives.targets],
											##'targetsY':[int(target.y) for target in objectives.targets],
											##'rad':[15 for start in objectives.starts],
											##'maxSpeed':[1 for start in objectives.starts]]}
								#})
	print args
	solverWrapper = SolverWrapper(args)
	plan = solverWrapper.callSolver()
	return MultiPlannerResponse(plan)
		
def formatMissionString(objectives,radius,multiplanner_map_scale):
	missionString = ''
	for i in range(len(objectives.starts)):
		missionString = missionString + '%s,%s:%s,%s:%s:%s;' %(int(round(objectives.starts[i].x*multiplanner_map_scale/(5))*(5)), 
															int(round(objectives.starts[i].y*multiplanner_map_scale/(5))*(5)),
															int(objectives.targets[i].x*multiplanner_map_scale),
															int(objectives.targets[i].y*multiplanner_map_scale),
															int(radius*multiplanner_map_scale),
															1)
	missionString = missionString[:-1]
	return missionString
	
if __name__ == '__main__':
	#rospy.init_node('aw_multi_solver_wrapper', anonymous=True)	
	rospy.init_node('aw_multi_service', anonymous=True)	
	
	#command_line = raw_input()
	#args = shlex.split(command_line)
	

	#//args = ['java', '-jar', '%s/uber-solver.jar' %(dirPath,)]
	#paths,error = wrapper(args)
	#//filePath = dirPath + '/paths.txt'
	rospy.loginfo("aw_multi_solver_wrapper: Waiting for service getObjectives")
	rospy.wait_for_service('getObjectives')

	#solverWrapper = SolverWrapper(jsonObjectives)
	
	s = rospy.Service('getMultiPlan', MultiPlanner, multiPlannerService )
	rospy.loginfo("aw_multi_solver_wrapper: Providing service getMultiPlan")
	rospy.spin()
	#filePath = os.path.join( 'paths.txt')
	#with open(filePath, 'w') as content_file:
		#dronePaths = content_file.write(paths)
		
	
