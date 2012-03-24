from math import atan, atan2, pi, radians, degrees, copysign, sin, cos, hypot
import numpy as np

#Robot
max_robot_speed = 0.8 # in [m/s]
max_robot_rot_speed = 2.6 # in [rad/s]

#Scan
scan_init = True
scan_pending = False
scan_finished = False
scan_rotation = 0
scan_orientation_start = 0
scan_orientation_max = 15.0
scan_orientation_min = 0.2
scan_length = 0
scan_speed = 1.0 # in [rad/s]
scan_orientation=[]
scan_range=[]

#FollowWall
follow_init = True 
follow_start = False 
first_outside_corner = True
curvature = pi/4 #in [rad]
perpendicular_orientation = -pi/2
angle_offset = radians(45) # orientation compare to the wall [0-pi/2]
curvature_rebound = radians(45) # in [rad]
threshold_dist = 0.5 # distance between the robot and the wall in [m]
threshold_rebound = 0.1 # in [m]
threshold_outside_corner = 2.0 # in [m]
threshold_inside_corner = -0.1 # in [m]
start_curve = 0
follow_step = 0

#WallEstimation
estimation_index = 0
estimation_data_length = 70
estimation_wall_x = np.zeros(estimation_data_length)
estimation_wall_y = np.zeros(estimation_data_length)
estimation_pose_x = np.zeros(estimation_data_length)
estimation_pose_y = np.zeros(estimation_data_length)
estimation_distance = np.zeros(estimation_data_length)
filenames=[0,0,0]
filenames[0]='WallEstimation/wallestimationlog'
filenames[1]=str(0)
filenames[2]='.txt'
index_log = 0

def Algorithm(pose_x, pose_y, heading, distance):
	if scan_finished == False:
		StaticScan(heading, distance)
		speed=0
		rot_speed=scan_speed
		direction = 0
	else:
		#follow wall
		speed=0
		rot_speed=0
		direction=0
		speed, rot_speed, direction = FollowWall(pose_x, pose_y, heading, distance)
	return speed, rot_speed, direction

def StaticScan(heading,distance):
	global scan_init
	global scan_pending
	global scan_finished
	global scan_rotation
	global scan_orientation_start
	global scan_orientation_max
	global scan_orientation_min
	global scan_length
	global scan_orientation
	global scan_range
	global perpendicular_orientation
	
	if scan_init == True:
		scan_init = False
		scan_pending = True
		scan_rotation=0
		scan_orientation=[]
		scan_range=[]
		scan_orientation_start = heading
		scan_orientation.append(heading)#Store the first element
		scan_range.append(distance)
		print 'Scan start'
	elif scan_pending == True:
		if abs(scan_rotation) < 2*pi:#scan
			scan_rotation=scan_rotation+Normalize(heading-scan_orientation[-1])
			scan_orientation.append(heading)
			scan_range.append(distance)
			print '\t\t\t',degrees(scan_rotation)
		else: #end of the scan
			scan_init=True
			scan_pending = False
			scan_finished = True
			scan_distance_max = max(scan_range)
			scan_orientation_max = scan_orientation[scan_range.index(scan_distance_max)]
			scan_distance_min = min(scan_range)
			scan_length = len(scan_range)
			scan_orientation_min = scan_orientation[scan_range.index(scan_distance_min)]
			perpendicular_orientation = scan_orientation_min #the perpendicular is the min of the scan
			print 'Scan min %0.2fm at %ddegrees\tmax %dm at %ddegrees' % ( scan_distance_min,degrees
			(scan_orientation_min),scan_distance_max,degrees(scan_orientation_max) )
	else:
		print '\n\n\t\t\t*** Error scan ***\n\n'
	
def FollowWall(pose_x, pose_y, heading, distance):
	rot_speed = 0
	speed = max_robot_speed*0.7
	direction = 0
	global follow_init
	global follow_start
	global first_outside_corner
	global curvature
	global perpendicular_orientation
	global angle_offset
	global curvature_rebound 
	global threshold_dist
	global threshold_rebound
	global threshold_outside_corner
	global threshold_inside_corner
	global start_curve
	global estimation_index
	global follow_step
	
	if follow_init==True:
		diff_orientation = Normalize(heading-perpendicular_orientation-angle_offset)
		print'Follow Init: %d - %d - offset = %d' % ( degrees(heading), degrees(perpendicular_orientation), degrees(diff_orientation) )
		if abs(diff_orientation) > 0.1:
			rot_speed = -diff_orientation*4 # P controller
			speed = 0
		else:
			follow_init = False
			follow_start = True
	elif follow_start == True:
		diff_dist = distance - threshold_dist
		if diff_dist > threshold_outside_corner: #Outside corner
			if first_outside_corner ==True:
				first_outside_corner=False
				curvature=0.3
				print 'Outside Corner first time'
			else:
				rot_speed = -0.7
				print 'Outside Corner'
			estimation_index=0 #reset the counter
		elif diff_dist < threshold_rebound:
			if diff_dist < threshold_inside_corner: # in [m]: #Inside corner
				speed = 0.2
				rot_speed = 0.6
				curvature = pi/2+angle_offset # go back
				print 'Inside corner'
				#start_curve = 0
			else:
				if start_curve > 30: # //last step was already the start of the curve to avoid problem after inside corner
					rot_speed = 0.5
					speed = 0.3
					print 'Heading Correction'
				else:
					curvature = curvature_rebound
					speed = 0.2
					print 'Start a curve ' 
					start_curve=start_curve+1
				first_outside_corner=True #reset
			estimation_index = 0 #reset the counter
		else: #do the curve and estimate the position of the wall
			curvature = curvature-0.005
			if curvature < -pi/3:
				curvature = -pi/3
			if distance < threshold_rebound+0.1: #slow down before next rebound
				speed = 0.2
			#rot_speed = HeadingControl( heading, Normalize(perpendicular_orientation + angle_offset) )
			print 'Do the curve'
			if follow_step == 0:
				follow_step = 0 
				start_curve = 0
				perpendicular = WallEstimation(distance, heading, pose_x, pose_y)# estimate the position of the wall
				if perpendicular != None:
					perpendicular_orientation = perpendicular
					print 'Perpendicular estimation is %d [degrees]' % degrees(perpendicular_orientation)
					if abs(Normalize(heading-(perpendicular_orientation+angle_offset))) > 0.15:#correct the bearing of the robot if the robot has not the correct offset 
						print 'New Perpendicular is %d [degrees]' % degrees(perpendicular_orientation)
						follow_init = True
						follow_start = False
					estimation_index = 0
			else:
				follow_step = follow_step + 1
		direction = pi/2 - angle_offset + curvature
		print '\t\tFollow wall: direction %d \t Rotspeed %0.2f\t Heading %d\t Distance %0.2f\t Perp. %0.1f' % (degrees(direction), rot_speed, degrees(heading),distance, degrees(perpendicular_orientation))
	else:
		print 'Error Follow_wall'
	return speed, rot_speed, direction
	
def WallEstimation(distance, heading, pose_x, pose_y):
	global estimation_wall_x
	global estimation_wall_y
	global estimation_pose_x
	global estimation_pose_y
	global estimation_index
	global estimation_data_length
	global perpendicular_orientation
	global estimation_distance
	global index_log
	global filenames

	estimation_wall_x[estimation_index] = distance*cos(heading)#store the current data
	estimation_wall_y[estimation_index] = distance*sin(heading)
	estimation_pose_x[estimation_index] = pose_x
	estimation_pose_y[estimation_index] = pose_y
	estimation_distance[estimation_index] = distance
	estimation_index=estimation_index+1
	if estimation_index>estimation_data_length-1:
		estimation_index=0
		
		wallestimationlog= ''.join(filenames)
		store_estimation = open(wallestimationlog,"w")
		filenames[1]=str(index_log)
		index_log=index_log+1
		for n in range(estimation_data_length):
			store_estimation.write("%s \t%s \t %s \t %s \t %s\n" % (estimation_wall_x[n], estimation_wall_y[n], estimation_pose_x[n], estimation_pose_y[n], estimation_distance[n]) )
		
		for n in range(estimation_data_length):#adapt the data in the current frame of the robot
			tX = pose_x - estimation_pose_x[n]#translation in the global frame,centered on the robot
			tY = pose_y - estimation_pose_y[n]
			estimation_wall_x[n] = -tX + estimation_wall_x[n]
			estimation_wall_y[n] = -tY + estimation_wall_y[n]
			estimation_pose_x[n] = pose_x
			estimation_pose_y[n] = pose_y
		
		store_estimation.write("\n\n") 
		for n in range(estimation_data_length):
			store_estimation.write("%s \t%s \n" % (estimation_wall_x[n], estimation_wall_y[n]) )
		
		parameter_a, parameter_b = LeastSquare(estimation_wall_x, estimation_wall_y,estimation_data_length) #estimate the wall
		perpendicular_orientation = atan(-1/parameter_a) #compute the perpendicular direction to the wall
		
		store_estimation.write("\n\n %s\t %s\t %s\t %s\t %s\n" % ( parameter_a, parameter_b, perpendicular_orientation, heading, distance) ) 
		store_estimation.close()
		
		if Normalize(perpendicular_orientation-heading) > pi/2: #the perpendicular direction cannot be outside of [-PI/2, PI/2] relatively to the robot
			perpendicular_orientation=Normalize(perpendicular_orientation-pi)
		elif Normalize(perpendicular_orientation-heading) < -pi/2:
			perpendicular_orientation=Normalize(perpendicular_orientation+pi)
		return perpendicular_orientation
	else:
		return None
	
def LeastSquare(x, y, length):
	# Fit a line, y = ax + b, through some noisy data-points:
	# We rewrite the line equation as y = Ap, where A = [[x 1]] and p = [[a], [b]].  Now use lstsq to solve for p:
	A = np.vstack([x, np.ones(length)]).T
	a, b = np.linalg.lstsq(A, y)[0]
	return a,b
	
def DistanceControl(distance, direction, speed):
	targetDistance = 0.7
	diff = distance - targetDistance
	if diff < 1 :
		Kp = 0.4
	else:
		Kp = 0.2
	 
	speed_x = diff * Kp + speed*cos(direction)
	if speed_x >speed:
		speed_x = speed
	elif speed_x < -speed:
		speed_x = -speed
	
	speed_y = speed*sin(direction)
	direction = atan2(speed_y, speed_x)
	speed = hypot(speed_x, speed_y)
	
	return speed, direction
	
def HeadingControl(heading, targetHeading):
	diff =  targetHeading - heading
	Kp = 2  
	rot_speed = diff * Kp 
	
	if rot_speed > max_robot_rot_speed:
		rot_speed = max_robot_rot_speed
	elif rot_speed < -max_robot_rot_speed:
		rot_speed = -max_robot_rot_speed
	
	return rot_speed
	
def Normalize(angle):
	while (angle<=-pi or angle>pi):
		angle=angle-copysign(2*pi,angle)# equivalent to angle-=2*pi*sign(angle)
	return angle	
