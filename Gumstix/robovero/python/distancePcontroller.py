from adc2 import LaserRead, ConsumptionRead
from uart2 import SendToArduino, ReceiveFromArduino

import time

#-----------------------------

#Parameters for the control
targetDistance = 1
Kp = 0.3

time.sleep(1) # wait before start
print 'hello'
while True:
	time_start=time.time()
	#speed = float(raw_input("Enter speed: "))
	#rot_speed = float(raw_input("Enter rot_speed: "))
	#orientation = float(raw_input("Enter orientation: "))
	#print'You have entered:',speed,rot_speed,orientation
	
	#read laser sensor
	distance = LaserRead()
	time_laser=time.time()
	 
	#read consumption: voltage and current
	current, voltage = ConsumptionRead()
	time_consumption=time.time()
	
	#P control
	diff = distance - targetDistance
	if diff < 1 :
		Kp = 0.5
	else:
		Kp = 0.3
	 
	speed = diff * Kp 
	
	if speed > 0.8: # max speed is 0.8 m/s
		speed = 0.8
	elif speed < -0.8:
		speed = -0.8
	
	rot_speed = 0
	orientation = 0
	time_controller=time.time()
	
	#send commands to Arduino with UART
	SendToArduino(speed, rot_speed, orientation)
	time_UART_send=time.time()
	
	#receive UART
	wheel=ReceiveFromArduino()
	time_UART_receive=time.time()
	
	loop_time=time.time()-time_start
	print 'Dist \t Volt \t Current \t Speed \t Rot. \t Orie \t Tlas \t Tcons \t Tctr \t Tsend \t Trece \t Ttot \t LoopFreq'
	print '%0.2f \t' % distance,
	print '%0.2f V \t %0.2f mA \t' % (voltage, current*1000),
	print '%0.2f \t %0.2f \t %0.2f \t' % (speed, rot_speed,orientation),
	print '%0.3f \t %0.3f \t %0.3f \t %0.3f \t  %0.3f \t %0.3f \t %0.1f Hz \n' % (time_laser-time_start, time_consumption-time_laser, time_controller-time_consumption, time_UART_send-time_controller, time_UART_receive-time_UART_send, loop_time, 1.0/loop_time)
