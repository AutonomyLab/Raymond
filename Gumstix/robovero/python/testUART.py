from uart2 import UARTInit, SendToArduino, ReceiveFromArduino, UARTSendandReceive
import time, random, serial
from robovero.extras import roboveroConfig

#-----------------------------



print 'Test UART'

print 'Robot configuration'
roboveroConfig()
UARTInit()
i=0
n=0
orientation=0
loop_time=0
while True:
	time_start=time.time()
	speed=2
	rot_speed = 10
	orientation= 0
	odom=UARTSendandReceive(speed, rot_speed, orientation)
	print 'Odom',odom
	loop_time=time.time()-time_start
	print "\nTime %0.3f s\tFrequency:%0.2f Hz \t" % (loop_time, 1/loop_time)
	
