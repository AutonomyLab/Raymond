from uart2 import SendToArduino
from robovero.extras import roboveroConfig
import time

#-----------------------------
# Initialize pin select registers
roboveroConfig()

#send commands to Arduino with UART
SendToArduino(0,0,0)
print 'Motors stopped'
time.sleep(1)

exit()




