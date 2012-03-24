from robovero.extras import roboveroConfig
from adc2 import LaserRead
import time

#-----------------------------
roboveroConfig()
print '\nTestLaser'


while True:
		
	distance = LaserRead()
	print 'Distance in meter\t', distance
	#time.sleep(0.1)
