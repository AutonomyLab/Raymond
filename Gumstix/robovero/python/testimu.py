from robovero.internals import robocaller
from robovero.extras import roboveroConfig
from IMU3 import IMURead, IMUInit
import time

#-----------------------------

print 'hello'

roboveroConfig()

IMUInit()


while True:
	Acc, Gyro, Mag=IMURead()
	print Acc,'\t', Gyro,'\t', Mag
	#print "%0.2f\t%0.2f\t%0.2f" % (Acc[0],Acc[1],Acc[2])
	
	#print "\nTime %0.2f \tFrequency:%0.2f Hz \t" % (loop_time, 1/loop_time)
	#print loop_time
