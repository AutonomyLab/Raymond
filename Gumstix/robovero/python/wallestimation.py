from algorithm import WallEstimation
import numpy as np
from math import pi, degrees, sin, cos
import matplotlib.pyplot as plt

length =30
wall_x= np.zeros(length)
wall_y= np.zeros(length)
for n in range(length):
	wall_x[n] = n*0.5
heading = pi/4
pose_x = [n*0.5 for n in range(length)]
pose_y = [(n-15)*(n-15)/100.0-3+0.05*n for n in range(length)]
distance = [pose_y[n]/sin(heading) for n in range(length)]

for n in range(length):
	perpendicular_orientation = WallEstimation(distance[n], heading, pose_x[n], pose_y[n])

print 'Perp. final',degrees(perpendicular_orientation)

a = float(raw_input('Param_a'))
b = float(raw_input('Param_b'))
print a,b
x = np.array([0,15])
# Plot the data along with the fitted line:
plt.figure()
plt.plot(wall_x,wall_y, label='Exact Wall')
plt.plot(pose_x,pose_y, 'x', label='Pose of the robot', markersize=5)
plt.plot(x, a*x + b, 'r', label='Estimated wall')
plt.legend()
plt.show()
