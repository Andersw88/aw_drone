
import os,csv
import numpy,pylab
from pprint import pprint

from operator import itemgetter
from itertools import groupby

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
	
if __name__ == '__main__':
	rv = resultsVisualizer('../../results-used.csv')
	#pprint(rv.resultsList)

	print [x for x in rv.generateFilterdResult(lambda x: float(x[4]) == 0)]
	values = []
	values.append(numpy.mean([float(x[6]) for x in rv.generateFilterdResult(lambda x: x[3] == '0' and x[5] == 'True')]))
	values.append(numpy.mean([float(x[7]) for x in rv.generateFilterdResult(lambda x: x[3] == '0' and x[5] == 'True')]))
	values.append(numpy.mean([float(x[6]) for x in rv.generateFilterdResult(lambda x: x[3] == '2' and x[5] == 'True')]))
	values.append(numpy.mean([float(x[7]) for x in rv.generateFilterdResult(lambda x: x[3] == '2' and x[5] == 'True')]))
	values.append(numpy.mean([float(x[6]) for x in rv.generateFilterdResult(lambda x: x[3] == '1' and x[5] == 'True')]))	
	values.append(numpy.mean([float(x[7]) for x in rv.generateFilterdResult(lambda x: x[3] == '1' and x[5] == 'True')]))
	

	pprint (values)
	values2 = [float(x[6]) for x in rv.generateFilterdResult(lambda x: x[3] == '0' and x[5] == 'True')]
	
	pylab.bar(range(0,6), values,width=0.5)
	#pylab.show()
	
	errors = []
	results = [x for x in rv.generateFilterdResult(lambda x: x[5] == 'True')]
	#pprint(results)
	results.sort(key=itemgetter(1))
	glo = [[x for x in g] for k,g in  groupby(results,key=itemgetter(1))]
	pprint(glo)
	
	#mu, sigma = 200, 25
	#x = mu + sigma*pylab.randn(10000)

	## the histogram of the data with histtype='step'
	#n, bins, patches = pylab.hist(x, 50, normed=1, histtype='stepfilled')
	#pylab.setp(patches, 'facecolor', 'g', 'alpha', 0.75)

	## add a line showing the expected distribution
	#y = pylab.normpdf( bins, mu, sigma)
	#l = pylab.plot(bins, y, 'k--', linewidth=1.5)
	