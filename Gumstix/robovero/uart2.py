"""Send user messages out of UART1.
"""
from robovero.extras import Array
from robovero.internals import robocaller
from robovero.LPC17xx import LPC_UART1
from robovero.lpc17xx_uart import UART_Send, UART_Receive, UART_FIFO_CFG_Type, UART_FIFOConfig, UART_FIFOConfigStructInit
from robovero.lpc_types import TRANSFER_BLOCK_Type
import struct ,time

reception_length = 18 #maximum number of bytes received
#msg = Array(18, 1, "")

numbers_byte=[0,0,0,0]
numbers_int=[0,0,0,0]
odom=[0,0,0,0]

def UARTInit():
	print 'UART configuration ...',
	UARTFIFOConfigStruct = UART_FIFO_CFG_Type()
	UART_FIFOConfigStructInit(UARTFIFOConfigStruct.ptr)
	UART_FIFOConfig(LPC_UART1, UARTFIFOConfigStruct.ptr)
	print 'done'
	
def SendToArduino(speed, rot_speed, orientation):
	value2send = FloatToBytes(speed, rot_speed, orientation)
	msg_send = Array(len(value2send),1, value2send)
	UART_Send(LPC_UART1, msg_send.ptr, msg_send.length, TRANSFER_BLOCK_Type.BLOCKING)	
#	print '  Byte sent:', byteSent,'\n'
	
def ReceiveFromArduino():
	length = UART_Receive(LPC_UART1, msg.ptr, reception_length, TRANSFER_BLOCK_Type.NONE_BLOCKING)
	print 'Reception: %d bytes:' % length,
	if length>=9 :
		message=[]#clear this array
		for n in range(length):#store the data
			message.append(msg[n])
			#print message[n],
			#if n==8:
				#print ''
		for n in range(4):#convert the data to int
			odom[n]=BytesToInt16(message[2*n],message[2*n+1])
		return odom
	else:
		return 'No data'
		
def UARTSendandReceive(speed, rot_speed, orientation):
	message_send = FloatToBytes(speed, rot_speed, orientation)
	message_receive=robocaller("UARTSendandReceive","int",message_send)
	print "Receive",message_receive
	if len(message_receive)>=9:
		for n in range(4):#convert the data to int
			odom[n]=BytesToInt16(message_receive[2*n],message_receive[2*n+1])
		return odom
	else:
		return 'No data'
		
def UARTConvert(message_receive):
	length=len(message_receive)
	if length >= 9:
		try:
			shift=message_receive[8:].index(69)
		except:
			print 'Delimiter 69 not found'
			return 0
		#if shift != 0:	
			#print 'Shift = ',shift
		for n in range(4):#convert the data to int
			odom[n]=BytesToInt16(message_receive[2*n+shift],message_receive[2*n+1+shift])*10e-7
		return odom
	else:
		print 'No Data'
		return 0
	
#------------------------------------

def floatToRawLongBits(value):
	return struct.unpack('l', struct.pack('f', value))[0]

def longBitsToFloat(bits):
	return struct.unpack('f', struct.pack('l', bits))[0]		
		
def FloatToBytes(number1, number2, number3):
	Bytes = [] # clear this array
	#Byteshex = []
	numbers=[number1,number2,number3]
	for j in range(3):
		numbers_string = hex( floatToRawLongBits( numbers[j] ) & 0xffffffff )[:-1] # convert float to hex representation. [:-1] remove the last character L, and 0xffffffff is uses for the negative number to avoid the minus sign before the hex number
		if numbers[j]==0:
			numbers_string  = "00000000" # to be sure that the number will have 4 bytes
		else:
			numbers_string  = numbers_string[len(numbers_string)-8:]#remove the prefix 0x
		#print '  Hex string:',numbers_string,'  ',
		for i in range(4):
			numbers_byte[i] = numbers_string[i*2:i*2+2]
			#Byteshex.append(numbers_byte[i])
			#Byteshex.append(' ')
			numbers_int[i] = int(numbers_byte[i][0],16)*16 + int(numbers_byte[i][1],16)
	#	print '  Byte:', numbers_byte,'  ',
	#	print '  Int:', numbers_int,'  ',
	#Byteshex.append(', ')
		Bytes= Bytes + numbers_int
#		print '  Value2send',Bytes
	Bytes.append(69) # add end of communication character
	#Byteshex.append('45')
	
	#print '  Value2send int',Bytes
	#print '  hex: ','[',''.join(Byteshex),']'
	return Bytes		
		
def BytesToInt16(byte1,byte2):
	IntNumber=256*ord(chr(byte1)) + ord(chr(byte2))
	if IntNumber>32767:
		IntNumber=-65536+IntNumber
	return IntNumber
