
import os,csv
import numpy,pylab
from pprint import pprint

from operator import itemgetter
from itertools import groupby
import sqlite3
from scipy import stats

class SqliteInteface():
	
	def __init__(self,relDBPath):
		dbPath = os.path.join(os.path.dirname(__file__),relDBPath)
		conn = sqlite3.connect(dbPath)
		self.c = conn.cursor()
		
	def __del__(self):
		self.c.close()


class resultsVisualizer():
	
	def __init__(self,relFilePath):
		filePath = os.path.join(os.path.dirname(__file__),relFilePath)
		self.openResultsFile(filePath)
	
	def openResultsFile(self,filePath):
		with open(filePath, 'rb') as csvfile:
			dialect = csv.Sniffer().sniff(csvfile.read(1024))
			csvfile.seek(0)
			self.reader = csv.reader(csvfile, dialect)
			self.parseResultsToList()
			
	
	def parseResultsToList(self):
		self.resultsList = [result for result in self.reader]
		#self.resultsList = [[int(result[0]),int(result[1]),result[2],int(result[3]),float(result[4]),bool(result[5]),int(result[6]),float(result[7])] for result in self.reader]

	def generateFilterdResult(self,resultFilter):
		for result in self.resultsList:
			if(resultFilter(result)):
				yield result
				
def autolabelP(rects):
	for rect in rects:
		height = rect.get_height()
		pylab.text(rect.get_x()+rect.get_width()/2., 1.05*height, '{:.0%}'.format(height),ha='center', va='bottom')
		#pylab.text(rect.get_x()+rect.get_width()/2., 1.05*height, '%d'%int(height),ha='center', va='bottom')

def autolabelD(rects):
	for rect in rects:
		height = rect.get_height()
		pylab.text(rect.get_x()+rect.get_width()/2., 1.05*height, '%d'%int(height),ha='center', va='bottom')
		
def makeBarPlot(labels,means,stds,autolabeler):
	ind = numpy.arange(len(means[0]))
	width = 0.25
	colours = ['red','blue','green','yellow']
	pylab.figure()

	pylab.title('Algorithms')
	
	rects = []
	for i in range(len(means)):
		#print means[i],stds[i]
		rects.extend([x for x in pylab.bar(width*ind+i-width,means[i],width,color=colours,align='center',yerr=stds[i],ecolor='k')])
	pylab.ylabel('Time (sec)')
	pylab.xticks(range(len(means)),labels)
	autolabeler(rects)
	
if __name__ == '__main__':
	sq = SqliteInteface('../../results-current.sqlite')

	#sq.c.execute('''SELECT (SELECT COUNT(*) from runs WHERE map_name = "maze3" GROUP BY tdm),COUNT(*) FROM runs WHERE plan_success = 1 AND map_name = "maze3" GROUP BY tdm''')
	#result = sq.c.fetchall()
	#pylab.figure()
	#pylab.title('Algorithms')
	#colours = ['red','green','yellow']
	#ind = [0,0.25,0.5]
	#rects = pylab.bar(ind,[float(x[1])/float(x[0]) for x in result],0.25,color=colours,align='center')
	#for rect in rects:
		#height = rect.get_height()
		#pylab.text(rect.get_x()+rect.get_width()/2., height*0.95, '{:.0%}'.format(height),ha='center', va='bottom')
	
	
	#sq.c.execute('''SELECT (SELECT COUNT(*) from runs WHERE map_name = "maze4" GROUP BY tdm),COUNT(*) FROM runs WHERE plan_success = 1 AND map_name = "maze4" GROUP BY tdm''')
	#result = sq.c.fetchall()
	#pylab.figure()
	#pylab.title('Algorithms')
	#colours = ['red','green','yellow']
	#ind = [0,0.25,0.5]
	#rects = pylab.bar(ind,[float(x[1])/float(x[0]) for x in result],0.25,color=colours,align='center')
	#for rect in rects:
		#height = rect.get_height()
		#pylab.text(rect.get_x()+rect.get_width()/2., height*0.95, '{:.0%}'.format(height),ha='center', va='bottom')
	
	sq.c.execute('''SELECT runH.num_drones,runH.plan_execution_time,runH.total_travel_distance,runT.plan_execution_time,runT.total_travel_distance,runG.plan_execution_time,runG.total_travel_distance 
				FROM problem_setups ps 
				JOIN runs as runH ON runH.problem_id = ps.id AND runH.tdm = 0 
				JOIN runs as runT ON runT.problem_id = ps.id AND runT.tdm = 2
				JOIN runs as runG ON runG.problem_id = ps.id AND runG.tdm = 3
				WHERE ps.map_name = "maze3"
				AND runH.plan_success = 1 AND runT.plan_success = 1 AND runG.plan_success = 1
				AND runH.num_drones = runT.num_drones AND runH.num_drones = runG.num_drones''')
	
	result = sq.c.fetchall()
	droneNrs = [4,6,8,10]
	means = [[numpy.mean([x[2]/x[6] for x in result if x[0] == n]),numpy.mean([x[4]/x[6] for x in result if x[0] == n]),numpy.mean([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
	stds =  [[numpy.std([x[2]/x[6] for x in result if x[0] == n]),numpy.std([x[4]/x[6] for x in result if x[0] == n]),numpy.std([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
	makeBarPlot(droneNrs,means,stds,autolabelP)
	
	means = [[numpy.mean([x[1]/x[5] for x in result if x[0] == n]),numpy.mean([x[3]/x[5] for x in result if x[0] == n]),numpy.mean([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]
	#print numpy.percentile(means,[95],axis=0)
	stds =  [[numpy.std([x[1]/x[5] for x in result if x[0] == n]),numpy.std([x[3]/x[5] for x in result if x[0] == n]),numpy.std([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]
	makeBarPlot(droneNrs,means,stds,autolabelP)
	
	sq.c.execute('''SELECT runH.num_drones,runH.plan_execution_time,runH.total_travel_distance,runT.plan_execution_time,runT.total_travel_distance,runG.plan_execution_time,runG.total_travel_distance 
				FROM problem_setups ps 
				JOIN runs as runH ON runH.problem_id = ps.id AND runH.tdm = 0 
				JOIN runs as runT ON runT.problem_id = ps.id AND runT.tdm = 2
				JOIN runs as runG ON runG.problem_id = ps.id AND runG.tdm = 3
				WHERE ps.map_name = "maze4"
				AND runH.plan_success = 1 AND runT.plan_success = 1 AND runG.plan_success = 1
				AND runH.num_drones = runT.num_drones AND runH.num_drones = runG.num_drones''')
	
	result = sq.c.fetchall()
	droneNrs = [4,6,8,10]
	means = [[numpy.mean([x[2]/x[6] for x in result if x[0] == n]),numpy.mean([x[4]/x[6] for x in result if x[0] == n]),numpy.mean([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
	stds =  [[numpy.std([x[2]/x[6] for x in result if x[0] == n]),numpy.std([x[4]/x[6] for x in result if x[0] == n]),numpy.std([x[6]/x[6] for x in result if x[0] == n])] for n in droneNrs]
	#cl = stats.norm.interval(0.95, loc=means[0][0], scale=stds[0][0])
	#print cl,means[0][0]
	makeBarPlot(droneNrs,means,stds,autolabelP)
	
	means = [[numpy.mean([x[1]/x[5] for x in result if x[0] == n]),numpy.mean([x[3]/x[5] for x in result if x[0] == n]),numpy.mean([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]

	stds =  [[numpy.std([x[1]/x[5] for x in result if x[0] == n]),numpy.std([x[3]/x[5] for x in result if x[0] == n]),numpy.std([x[5]/x[5] for x in result if x[0] == n])] for n in droneNrs]
	makeBarPlot(droneNrs,means,stds,autolabelP)


	pylab.show()
	
	
	
	
