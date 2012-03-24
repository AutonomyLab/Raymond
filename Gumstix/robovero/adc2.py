"""Acquire and display ADC readings using the Arduino API.
"""

from robovero.arduino import analogRead, AD0_0, AD0_2, AD0_3, analogReadAll
from time import sleep

__author__ =			"Neil MacMunn"
__email__ =				"neil@gumstix.com"
__copyright__ = 	"Copyright 2010, Gumstix Inc"
__license__ = 		"BSD 2-Clause"
__version__ =			"0.1"


#ADC
MaxValue = 4095.0 #12bit, 2^12
MaxVoltage = 3.3 #3.3V

#Laser
laser_MinVoltage = 813.0 # need to be measured
laser_MaxVoltage = 4095.0 
laser_MinDistance = 0.2
laser_MaxDistance = 15.0
#Consumption - Data from the datasheet of the AttoPilot voltage and current sensor
VoltperVolt = 1/0.243 	 # Volt of the battery per volt from the sensor
AmpereperVolt = 1/0.0733 #Ampere of the battery per volt from the sensor 


  
def LaserRead():
	voltage = analogRead(AD0_0)
	distance = (voltage-laser_MinVoltage)/(laser_MaxVoltage-laser_MinVoltage)*(laser_MaxDistance-laser_MinDistance)+laser_MinDistance
	if distance < 0:
		distance = laser_MinDistance
	return(distance)
	
def DistanceInit(length):
	distance = 0
	number=10
	for n in range(number):
		distance =  distance + ReadAll()[0]
	distance = distance/float(number)
	distances = [ distance for n in range(length) ]
	return(distances)
	
def ConsumptionRead():
	voltage1=analogRead(AD0_3)#Battery current
	BatteryCurrent = voltage1*MaxVoltage/MaxValue * AmpereperVolt
	
	voltage2=analogRead(AD0_2)#Battery voltage
	BatteryVoltage = voltage2*MaxVoltage/MaxValue * VoltperVolt
	return BatteryCurrent, BatteryVoltage
	
def ReadAll():
	analogAll = analogReadAll()
#	print'AD0_0 %d\tAD0_2 %d\t AD0_3 %d' %(analogAll[0], analogAll[1], analogAll[2])
	distance = (analogAll[0]-laser_MinVoltage)/(laser_MaxVoltage-laser_MinVoltage)*(laser_MaxDistance-laser_MinDistance)+laser_MinDistance
	if distance < 0:
		distance = laser_MinDistance
		
	BatteryVoltage = analogAll[1]*MaxVoltage/MaxValue * VoltperVolt
	BatteryCurrent = analogAll[2]*MaxVoltage/MaxValue * AmpereperVolt	
	
	return distance, BatteryCurrent, BatteryVoltage

def ADCConvert(analogAll):
	distance = (analogAll[0]-laser_MinVoltage)/(laser_MaxVoltage-laser_MinVoltage)*(laser_MaxDistance-laser_MinDistance)+laser_MinDistance
	if distance < 0:
		distance = laser_MinDistance
		
	BatteryVoltage = analogAll[1]*MaxVoltage/MaxValue * VoltperVolt
	BatteryCurrent = analogAll[2]*MaxVoltage/MaxValue * AmpereperVolt	
	
	return distance, BatteryVoltage, BatteryCurrent
	
def BatteryVoltage():
	number = 20
	voltage = 0
	for n in range(number):
		distance, battery_current, battery_voltage = ReadAll()
		voltage = voltage + battery_voltage
	voltage = voltage/float(number)
	return(voltage)
		
