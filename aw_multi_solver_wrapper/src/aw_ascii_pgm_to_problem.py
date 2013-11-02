

import sys,os
from pprint import pprint

if __name__ == '__main__':
		

		filePath = os.path.join(os.path.dirname(__file__), sys.argv[1])
		with open(filePath, 'r') as content_file:
			maptext = content_file.read().split('\n')
		#print maptext
		scale = 10
		if(len(sys.argv) == 3):
			scale = int(sys.argv[2])
		
		mapSize = [ int(x) for x in maptext[2].split(' ') ]
		mapData = maptext[4:-1]
		
		#pprint (mapSize)
		#pprint (mapData)
		
		print"""<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<multiagentproblem>
  <environment>"""
		
		for i in range(0,mapSize[0]):
			for j in range(0,mapSize[1]):
				if(mapData[i+j*mapSize[0]] == "0"):
					x = i*scale
					y = (mapSize[1]-j)*scale
					#y = j*scale
					
					print """<obstacle>
<point id="0">
<x>%s</x>
<y>%s</y>
</point>
<point id="1">
<x>%s</x>
<y>%s</y>
</point>
<point id="2">
<x>%s</x>
<y>%s</y>
</point>
<point id="3">
<x>%s</x>
<y>%s</y>
</point>
</obstacle>""" %(x,y,x+scale,y,x+scale,y-scale,x,y-scale)

		print"""    <bounds>
      <point>
        <x>0</x>
        <y>0</y>
      </point>
      <point>
        <x>%s</x>
        <y>%s</y>
      </point>
    </bounds>""" %(mapSize[0]*scale,mapSize[1]*scale)
    
print """</environment>
  <agents/>
</multiagentproblem>"""