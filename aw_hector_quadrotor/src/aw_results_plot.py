
import os#,csv

from pprint import pprint

#from operator import itemgetter
#from itertools import groupby
import sqlite3
import numpy,pylab
from scipy import stats

class SqliteInteface():
	
	def __init__(self,relDBPath):
		dbPath = os.path.join(os.path.dirname(__file__),relDBPath)
		conn = sqlite3.connect(dbPath)
		self.c = conn.cursor()

		self.c.execute('''CREATE INDEX IF NOT EXISTS problemIndex ON runs (problem_id)''')
		conn.commit()
		
	def __del__(self):
		self.c.close()


#class resultsVisualizer():
	
	#def __init__(self,relFilePath):
		#filePath = os.path.join(os.path.dirname(__file__),relFilePath)
		#self.openResultsFile(filePath)
	
	#def openResultsFile(self,filePath):
		#with open(filePath, 'rb') as csvfile:
			#dialect = csv.Sniffer().sniff(csvfile.read(1024))
			#csvfile.seek(0)
			#self.reader = csv.reader(csvfile, dialect)
			#self.parseResultsToList()
			
	
	#def parseResultsToList(self):
		#self.resultsList = [result for result in self.reader]
		##self.resultsList = [[int(result[0]),int(result[1]),result[2],int(result[3]),float(result[4]),bool(result[5]),int(result[6]),float(result[7])] for result in self.reader]

	#def generateFilterdResult(self,resultFilter):
		#for result in self.resultsList:
			#if(resultFilter(result)):
				#yield result
				
def autolabelP(rects):
	for rect in rects:
		height = rect.get_height()
		pylab.text(rect.get_x()+rect.get_width()/2., 1.05*height, '{:.0%}'.format(height),ha='center', va='bottom')
		#pylab.text(rect.get_x()+rect.get_width()/2., 1.05*height, '%d'%int(height),ha='center', va='bottom')

def autolabelD(rects):
	for rect in rects:
		height = rect.get_height()
		pylab.text(rect.get_x()+rect.get_width()/2., 1.05*height, '%d'%int(height),ha='center', va='bottom')
		
def makeBarPlot(title,labels,means,stds,autolabeler):
	ind = numpy.arange(len(means[0]))
	width = 0.25
	colours = ['red','green','yellow']
	pylab.figure()

	pylab.title(title)
	
	rects = []
	for i in range(len(means)):
		#print means[i],stds[i]
		rects.extend([x for x in pylab.bar(width*ind+i-width,means[i],width,color=colours,align='center',yerr=stds[i],ecolor='k')])
	pylab.ylabel('Time (sec)')
	pylab.xticks(range(len(means)),labels)
	autolabeler(rects)
	
def mean_confidence_interval(data, confidence=0.95,pdist = stats.t):
	a = 1.0*numpy.array(data)
	n = len(a)
	m, se = numpy.mean(a), stats.sem(a)
	h = se * pdist._ppf((1+confidence)/2., n-1)
	return h
	
	
def plotTimeAndDistanceP(map_name,figNameT = None,figNameD = None):
	sq.c.execute('''SELECT runH.num_drones,runH.plan_execution_time,runH.total_travel_distance,runT.plan_execution_time,runT.total_travel_distance,runG.plan_execution_time,runG.total_travel_distance 
				FROM problem_setups ps 
				JOIN runs as runH ON runH.problem_id = ps.id AND runH.tdm = 0 
				JOIN runs as runT ON runT.problem_id = ps.id AND runT.tdm = 2
				JOIN runs as runG ON runG.problem_id = ps.id AND runG.tdm = 3
				WHERE ps.map_name = "%s"
				AND runH.plan_success = 1 AND runT.plan_success = 1 AND runG.plan_success = 1
				AND runH.num_drones = runT.num_drones AND runH.num_drones = runG.num_drones'''%map_name)
	
	result = sq.c.fetchall()
	droneNrs = [4,6,8,10]
	#pdist = stats.weibull_min
	means = [[numpy.mean([x[2]/x[6] for x in result if x[0] == n]),
			numpy.mean([x[4]/x[6] for x in result if x[0] == n]),
			numpy.mean([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
			
	#ci = [[mean_confidence_interval([x[2]/x[6] for x in result if x[0] == n]),
			#mean_confidence_interval([x[4]/x[6] for x in result if x[0] == n]),
			#mean_confidence_interval([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
	#stds =  [[numpy.std([x[2]/x[6] for x in result if x[0] == n]),numpy.std([x[4]/x[6] for x in result if x[0] == n]),numpy.std([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
	tsem = [[stats.tsem([x[2]/x[6] for x in result if x[0] == n]),stats.tsem([x[4]/x[6] for x in result if x[0] == n]),stats.tsem([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
	makeBarPlot("Distance",droneNrs,means,tsem,autolabelP)
	if(figNameD):
		pylab.savefig(figNameD,bbox_inches='tight')
		
	means = [[numpy.mean([x[1]/x[5] for x in result if x[0] == n]),
			numpy.mean([x[3]/x[5] for x in result if x[0] == n]),
			numpy.mean([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]
			
	#ci = [[mean_confidence_interval([x[1]/x[5] for x in result if x[0] == n]),
			#mean_confidence_interval([x[3]/x[5] for x in result if x[0] == n]),
			#mean_confidence_interval([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]
			
	#stds =  [[numpy.std([x[1]/x[5] for x in result if x[0] == n]),numpy.std([x[3]/x[5] for x in result if x[0] == n]),numpy.std([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]
	tsem = [[stats.tsem([x[1]/x[5] for x in result if x[0] == n]),stats.tsem([x[3]/x[5] for x in result if x[0] == n]),stats.tsem([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]
	makeBarPlot("Time",droneNrs,means,tsem,autolabelP)
	if(figNameT):
		pylab.savefig(figNameT,bbox_inches='tight')
		
def plotTimeAndDistancePLimit(map_name,order = "ASC", limit=20,figNameT = None,figNameD = None):
	sq.c.execute('''SELECT runH.num_drones,runH.plan_execution_time,runH.total_travel_distance,runT.plan_execution_time,runT.total_travel_distance,runG.plan_execution_time,runG.total_travel_distance 
				FROM problem_setups ps 
				JOIN runs as runH ON runH.problem_id = ps.id AND runH.tdm = 0 
				JOIN runs as runT ON runT.problem_id = ps.id AND runT.tdm = 2
				JOIN runs as runG ON runG.problem_id = ps.id AND runG.tdm = 3
				WHERE ps.map_name = "%s"
				AND runH.plan_success = 1 AND runT.plan_success = 1 AND runG.plan_success = 1
				AND runH.num_drones = runT.num_drones AND runH.num_drones = runG.num_drones
				ORDER BY runG.plan_execution_time %s '''%(map_name,order))
	
	result = sq.c.fetchall()
	print result
	droneNrs = [4,6,8,10]
	#pdist = stats.weibull_min
	means = [[numpy.mean([x[2]/x[6] for x in result if x[0] == n]),
			numpy.mean([x[4]/x[6] for x in result if x[0] == n]),
			numpy.mean([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
			
	#ci = [[mean_confidence_interval([x[2]/x[6] for x in result if x[0] == n]),
			#mean_confidence_interval([x[4]/x[6] for x in result if x[0] == n]),
			#mean_confidence_interval([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
	#stds =  [[numpy.std([x[2]/x[6] for x in result if x[0] == n]),numpy.std([x[4]/x[6] for x in result if x[0] == n]),numpy.std([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
	tsem = [[stats.tsem([x[2]/x[6] for x in result if x[0] == n]),stats.tsem([x[4]/x[6] for x in result if x[0] == n]),stats.tsem([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
	makeBarPlot("Distance %s"%order,droneNrs,means,tsem,autolabelP)
	if(figNameD):
		pylab.savefig(figNameD,bbox_inches='tight')
		
	means = [[numpy.mean([x[1]/x[5] for x in result if x[0] == n]),
			numpy.mean([x[3]/x[5] for x in result if x[0] == n]),
			numpy.mean([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]
			
	#ci = [[mean_confidence_interval([x[1]/x[5] for x in result if x[0] == n]),
			#mean_confidence_interval([x[3]/x[5] for x in result if x[0] == n]),
			#mean_confidence_interval([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]
			
	#stds =  [[numpy.std([x[1]/x[5] for x in result if x[0] == n]),numpy.std([x[3]/x[5] for x in result if x[0] == n]),numpy.std([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]
	tsem = [[stats.tsem([x[1]/x[5] for x in result if x[0] == n]),stats.tsem([x[3]/x[5] for x in result if x[0] == n]),stats.tsem([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]
	makeBarPlot("Time %s"%order,droneNrs,means,tsem,autolabelP)
	if(figNameT):
		pylab.savefig(figNameT,bbox_inches='tight')
		
def plotTimeAndDistanceD(map_name,figNameT = None,figNameD = None):
	sq.c.execute('''SELECT runH.num_drones,runH.plan_execution_time,runH.total_travel_distance,runT.plan_execution_time,runT.total_travel_distance,runG.plan_execution_time,runG.total_travel_distance 
				FROM problem_setups ps 
				JOIN runs as runH ON runH.problem_id = ps.id AND runH.tdm = 0 
				JOIN runs as runT ON runT.problem_id = ps.id AND runT.tdm = 2
				JOIN runs as runG ON runG.problem_id = ps.id AND runG.tdm = 3
				WHERE ps.map_name = "%s"
				AND runH.plan_success = 1 AND runT.plan_success = 1 AND runG.plan_success = 1
				AND runH.num_drones = runT.num_drones AND runH.num_drones = runG.num_drones'''%map_name)
	
	result = sq.c.fetchall()
	droneNrs = [4,6,8,10]
	means = [[numpy.mean([x[2] for x in result if x[0] == n]),
			numpy.mean([x[4] for x in result if x[0] == n]),
			numpy.mean([x[6] for x in result if x[0] == n])] for n in droneNrs]
			
	#ci = [[mean_confidence_interval([x[2] for x in result if x[0] == n]),
			#mean_confidence_interval([x[4] for x in result if x[0] == n]),
			#mean_confidence_interval([x[6] for x in result if x[0] == n])] for n in droneNrs]
	#stds =  [[numpy.std([x[2]/x[6] for x in result if x[0] == n]),numpy.std([x[4]/x[6] for x in result if x[0] == n]),numpy.std([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
	tsem = [[stats.tsem([x[2] for x in result if x[0] == n]),
				stats.tsem([x[4] for x in result if x[0] == n]),
				stats.tsem([x[6] for x in result if x[0] == n])] for n in droneNrs]
	makeBarPlot("Distances %s"%map_name,droneNrs,means,tsem,autolabelD)
	if(figNameD):
		pylab.savefig(figNameD,bbox_inches='tight')
		
	means = [[numpy.mean([x[1] for x in result if x[0] == n]),
			numpy.mean([x[3] for x in result if x[0] == n]),
			numpy.mean([x[5] for x in result if x[0] == n])] for n in droneNrs]
			
	#ci = [[mean_confidence_interval([x[1] for x in result if x[0] == n]),
			#mean_confidence_interval([x[3] for x in result if x[0] == n]),
			#mean_confidence_interval([x[5] for x in result if x[0] == n])] for n in droneNrs]
	#stds =  [[numpy.std([x[1]/x[5] for x in result if x[0] == n]),numpy.std([x[3]/x[5] for x in result if x[0] == n]),numpy.std([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]
	tsem =  [[stats.tsem([x[1] for x in result if x[0] == n]),
				stats.tsem([x[3] for x in result if x[0] == n]),
				stats.tsem([x[5] for x in result if x[0] == n])] for n in droneNrs]
	makeBarPlot("Time %s"%map_name,droneNrs,means,tsem,autolabelD)
	if(figNameT):
		pylab.savefig(figNameT,bbox_inches='tight')
		
def plotHist(map_name,figNameT = None,figNameD = None):
	sq.c.execute('''SELECT runH.num_drones,runH.plan_execution_time,runH.total_travel_distance,runT.plan_execution_time,runT.total_travel_distance,runG.plan_execution_time,runG.total_travel_distance 
				FROM problem_setups ps 
				JOIN runs as runH ON runH.problem_id = ps.id AND runH.tdm = 0 
				JOIN runs as runT ON runT.problem_id = ps.id AND runT.tdm = 2
				JOIN runs as runG ON runG.problem_id = ps.id AND runG.tdm = 3
				WHERE ps.map_name = "%s"
				AND runH.plan_success = 1 AND runT.plan_success = 1 AND runG.plan_success = 1
				AND runH.num_drones = runT.num_drones AND runH.num_drones = runG.num_drones'''%map_name)
	
	result = sq.c.fetchall()
	droneNrs = [4,6,8,10]
	pylab.figure()
	pylab.title("%s:Hist of timeP"%map_name)
	pylab.hist([x[2]/x[6] for x in result if x[0] == 4],20)
	pylab.figure()
	pylab.title("%s:Hist of time"%map_name)
	pylab.hist([x[2] for x in result if x[0] == 4],20)
	
	pylab.figure()
	pylab.title("%s:Hist of time"%map_name)
	pylab.hist([x[6] for x in result if x[0] == 4],20)

	
def plotSuccessP(map_name,figName = None):
	sq.c.execute('''SELECT (SELECT COUNT(*) from runs WHERE map_name = "%s" GROUP BY num_drones,tdm),COUNT(*) FROM runs WHERE plan_success = 1 AND map_name = "%s" GROUP BY num_drones,tdm'''%(map_name,map_name))
	result = sq.c.fetchall()
	pylab.figure()
	pylab.title('Algorithms')
	colours = ['red','green','yellow']
	ind = []
	for i in range(4):
		ind.extend(range(0+i*4,3+i*4))
	rects = pylab.bar(ind,[float(x[1])/float(x[0]) for x in result],1,align='center',color=colours)
	for rect in rects:
		height = rect.get_height()
		pylab.text(rect.get_x()+rect.get_width()/2., height*0.95, '{:.0%}'.format(height),ha='center', va='bottom')
	
	if(figName):
		pylab.savefig(figName,bbox_inches='tight')
		
		
def plotTimeAndDistanceLine(map_name,figNameT = None,figNameD = None):
	sq.c.execute('''SELECT runH.num_drones,runH.plan_execution_time,runH.total_travel_distance,runT.plan_execution_time,runT.total_travel_distance,runG.plan_execution_time,runG.total_travel_distance 
				FROM problem_setups ps 
				JOIN runs as runH ON runH.problem_id = ps.id AND runH.tdm = 0 
				JOIN runs as runT ON runT.problem_id = ps.id AND runT.tdm = 2
				JOIN runs as runG ON runG.problem_id = ps.id AND runG.tdm = 3
				WHERE ps.map_name = "%s"
				AND runH.plan_success = 1 AND runT.plan_success = 1 AND runG.plan_success = 1
				AND runH.num_drones = runT.num_drones AND runH.num_drones = runG.num_drones
				ORDER BY runG.plan_execution_time'''%map_name)
				
	result = sq.c.fetchall()
	droneNrs = [4,6,8,10]
	#pprint(result)
	#print [range(0,len(result)) for i in range(0,3)]
	#print zip(*[[x[2],x[4],x[6]] for x in result if x[0] == 10])
	#pylab.plot(zip(*[range(0,len(result)) for i in range(0,3)]),[[x[2],x[4],x[6]] for x in result if x[0] == 10])
	#print len(result),len([x[2] for x in result if x[0] == 10])
	
	
	#test = [[x[1],x[3],x[5]] for x in result if x[0] == 10]
	##test2 = [list(i) for i in zip(*test)]
	#ranged = [range(0,len(test)) for i in range(0,3)]
	#print len(ranged),len(ranged[0])
	#print len(test),len(test[0])
	#pylab.plot([range(0,3) for i in range(0,len(test))],test)
	#print test
	#print test2
	
	for nrDrones in droneNrs:
		pylab.figure()
		pylab.title("%s,%s"%(map_name,nrDrones))
		resultsfor = [x for x in result if x[0] == nrDrones]
		#print len(resultsfor)
		pylab.plot(range(0,len(resultsfor)),[[x[1],x[3],x[5]] for x in resultsfor])
		#pylab.plot(range(0,len(resultsfor)),[x[3] for x in resultsfor])
		#pylab.plot(range(0,len(resultsfor)),[x[5] for x in resultsfor])
		pylab.savefig("linePlot%s-%s.pdf"%(nrDrones,map_name),bbox_inches='tight')
		
	
def plotTimeScatter(map_name,figNameT = None,figNameD = None):
	sq.c.execute('''SELECT runH.num_drones,runH.plan_execution_time,runH.total_travel_distance,runT.plan_execution_time,runT.total_travel_distance,runG.plan_execution_time,runG.total_travel_distance 
				FROM problem_setups ps 
				JOIN runs as runH ON runH.problem_id = ps.id AND runH.tdm = 0 
				JOIN runs as runT ON runT.problem_id = ps.id AND runT.tdm = 2
				JOIN runs as runG ON runG.problem_id = ps.id AND runG.tdm = 3
				WHERE ps.map_name = "%s"
				AND runH.plan_success = 1 AND runT.plan_success = 1 AND runG.plan_success = 1
				AND runH.num_drones = runT.num_drones AND runH.num_drones = runG.num_drones
				ORDER BY runG.plan_execution_time'''%map_name)
				
	result = sq.c.fetchall()
	droneNrs = [4,6,8,10]
	
	for nrDrones in droneNrs:
		pylab.figure()
		#pylab.axis('equal')
		#pylab.ylim(ymin=0,ymax=2)
		pylab.title("%s,%s"%(map_name,nrDrones))
		resultsfor = [[x[1],x[3],x[1]/x[5],x[3]/x[5],x[5]] for x in result if x[0] == nrDrones]
		resultsfortans = [list(row) for row in zip(*resultsfor)]
		#print resultsfortans[0],resultsfortans[1],resultsfortans[2]
		#pylab.plot(range(0,len(resultsfor)),[[x[1],x[3],x[5]] for x in resultsfor])
		#pylab.scatter(resultsfortans[4],resultsfortans[2],marker='o')
		#pylab.scatter(resultsfortans[4],resultsfortans[3],marker='x')
		
		fit = pylab.polyfit(resultsfortans[4],resultsfortans[0],1)
		fit_fn = pylab.poly1d(fit)
		pylab.plot(resultsfortans[4], resultsfortans[0], 'ro', resultsfortans[4], fit_fn(resultsfortans[4]), '--r') 
		#print fit[:,:,1]
		
		fit = pylab.polyfit(resultsfortans[4],resultsfortans[1],1)
		fit_fn = pylab.poly1d(fit)
		pylab.plot(resultsfortans[4], resultsfortans[1], 'gx', resultsfortans[4], fit_fn(resultsfortans[4]), ':g') 
	
		pylab.savefig("scatterPlot%s-%s.pdf"%(nrDrones,map_name),bbox_inches='tight')
	#for nrDrones in droneNrs:
		#pylab.figure()
		#pylab.title("%s,%s"%(map_name,nrDrones))
		#resultsfor = [x for x in result if x[0] == nrDrones]
		#print len(resultsfor)
		#pylab.plot(range(0,len(resultsfor)),[[x[1],x[3],x[5]] for x in resultsfor])
		##pylab.plot(range(0,len(resultsfor)),[x[3] for x in resultsfor])
		##pylab.plot(range(0,len(resultsfor)),[x[5] for x in resultsfor])
		#pylab.savefig("linePlot%s-%s.pdf"%(nrDrones,map_name),bbox_inches='tight')
	
	
if __name__ == '__main__':
	sq = SqliteInteface('../../results-current.sqlite')

	plotSuccessP("maze3","maze3_success.pdf")
	plotSuccessP("maze4","maze4_success.pdf")
	plotSuccessP("liuB2","liuB2_success.pdf")
	
	plotTimeAndDistanceP("maze3","maze3_time.pdf","maze3_distance.pdf")
	plotTimeAndDistanceP("maze4","maze4_time.pdf","maze4_distance.pdf")
	plotTimeAndDistanceP("liuB2","liuB2_time.pdf","liuB2_distance.pdf")
	plotTimeAndDistancePLimit(map_name="liuB2",figNameT="liuB2_timeL_h.pdf",figNameD="liuB2_distanceL_h.pdf",order="DESC")
	plotTimeAndDistancePLimit(map_name="liuB2",figNameT="liuB2_timeL_l.pdf",figNameD="liuB2_distanceL_l.pdf",order="ASC")
	
	#plotTimeAndDistancePLimit(map_name="maze3",figNameT="maze3_timeL_h.pdf",figNameD="maze3_distanceL_h.pdf",order="DESC")
	#plotTimeAndDistancePLimit(map_name="maze3",figNameT="maze3_timeL_l.pdf",figNameD="maze3_distanceL_l.pdf",order="ASC")

	
	plotTimeAndDistanceD("maze3","maze3_timeD.pdf","maze3_distanceD.pdf")
	plotTimeAndDistanceD("maze4","maze4_timeD.pdf","maze4_distanceD.pdf")
	plotTimeAndDistanceD("liuB2","liuB2_timeD.pdf","liuB2_distanceD.pdf")
	plotHist("liuB2")
	plotTimeAndDistanceLine("liuB2")
	plotTimeScatter("liuB2")
	plotTimeScatter("maze3")
	plotTimeAndDistanceLine("maze3")
	pylab.show()
	
	
	
	
