from Point import *

import math
import pypot.robot


# CONSTANT DEFINITIONS

L1 = 51.0 #mm
L2 = 63.7 #mm
L3 = 93.0 #mm
ALPHA = math.radians(-20.69)
BETA = math.radians(5.06)


###########################################################
constL1 = 51
constL2 = 63.7
constL3 = 93
# Angle to match the theory with reality for theta 2 (measures of the triangle are 22.5, 60.7, 63.7). => Angle =  -20.69
theta2Correction = -20.69
# Same goes for theta 3 : +90 - 20.69 - a. Where a = asin(8.2/93) = 5.06
theta3Correction = 90 + theta2Correction - 5.06

# Given the sizes (a, b, c) of the 3 sides of a triangle, returns the angle between a and b using the alKashi theorem.
def alKashi(a, b, c):
	value = ((a*a)+(b*b)-(c*c))/(2*a*b)
	#Note : to get the other altenative, simply change the sign of the return :
	return -math.acos(value)

def computeIK(x, y, z, l1=constL1, l2=constL2,l3=constL3) :
	# theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
	theta1 = math.atan2(y, x)

	# Distance between the second motor and the projection of the end of the leg on the X/Y plane
	xp = math.sqrt(x*x+y*y)-l1
	if (xp < 0) :
		print("Destination point too close")
		xp = 0

	# Distance between the second motor arm and the end of the leg
	d = math.sqrt(math.pow(xp,2) + math.pow(z,2))
	if (d > l2+l3):
		print("Destination point too far away")
		d = l2+l3

	# Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
	theta2 = alKashi(l2, d, l3) - math.atan2(z, xp)
	theta3 = math.pi - alKashi(l2, l3, d)

	return [modulo180(math.degrees(theta1)), modulo180(math.degrees(theta2) + theta2Correction), modulo180(math.degrees(theta3) + theta3Correction)]

def modulo180(angle) :
	if (-180 < angle < 180) :
		return angle

	angle  = angle % 360
	if (angle > 180) :
		return -360 + angle

	return angle

#####################################################


def leg_ik(x,y,z,l1=L1, l2=L2, l3=L3, alpha=ALPHA, beta=BETA):
	theta1 = math.atan2(y, x)
	
	p1 = {
		"x": l1 * math.cos(theta1),
		"y": l1 * math.sin(theta1),
		"z": 0
	}
	
	d13 = math.sqrt( (x - p1["x"])**2 + (y - p1["y"])**2 )
	
	a = math.atan2(z, d13)
	
	d = math.sqrt( ((x - p1["x"])**2) + ((y - p1["y"])**2) + ((z - p1["z"])**2) )
	
	# in the case of a planar angle, theta3 is equal to 0 because the d length is equal to l2 + l3
	if x==0:
		theta3=0
		theta3C=0
		b=0
		theta2=0
		theta2C=0
	else:
		theta3 = math.radians(180) - math.acos( (l2**2 + l3**2 - d**2) / (2 * l2 * l3))
		theta3C = theta3 + alpha + beta - math.radians(90)
		b = math.acos((l2**2 + d**2 - l3**2) / (2 * l2 * d) )
		theta2 = b+a
		theta2C = -theta2 - alpha
	return [math.degrees(theta1), math.degrees(theta2C), math.degrees(theta3C)]


def smooth_step(x, base=10):
	'''
		this function is like a cosine, it returns a number 
		between 0 and 1 that represent the positive part
		of a cosine softened to match with Z position of a
		walking leg. 
	'''
	return math.log(max(1, 1 + math.cos(x) * base), base)

def rotateXY(pos, angle):
	'''
		Rotate the point at position pos arround the 0,0,0
		point with angle in degrees. Rotation arround Z axis.
	'''
	theta = math.radians(angle)
	return Point(
		(math.cos(theta)*pos.x) + (-math.sin(theta)*pos.y),
		(math.sin(theta)*pos.x) + (math.cos(theta)*pos.y),
		pos.z
	)

def posOnCircle(angle, diameter, height=0):
	'''
		Calculate the position of a point on a circle of diameter
		at the height. The circle is defined arround 0.0.0 point
		arround Z axis.
	'''
	theta = math.radians(angle)
	return Point(
		math.cos(theta) * diameter/2.0,
		math.sin(theta) * diameter/2.0,
		height
	)

def step(i):
	'''
		Calculate the height of a leg at a T instant
	'''
	return Point(
		0,
		0,
		math.log(max(1, 1 + math.cos(i)*10),10)
	)

def calc_leg(id, coord):
	'''
		Normalize the position of each leg arround the robot
		to match with the center of the robot.
		Thanks to it, we only put a position from center and
		a leg identifier to move a leg without take in count
		his own referential.
	'''
	new = Point()	
	if id==1:
		new.x = -coord.y - 102
		new.y = coord.x
	elif id==2:
		new = coord.copy()
		new.y = new.y + 24
		new.x = new.x - 74
	elif id==3:
		new = coord.copy()
		new.y = new.y - 24
		new.x = new.x - 74
	elif id==4:
		new.x = coord.y - 102
		new.y = -coord.x
	elif id==5:
		new = -coord.copy()
		new.y = new.y + 24
		new.x = new.x - 74
	elif id==6:
		new = -coord.copy()
		new.y = new.y - 24
		new.x = new.x - 74
	new.z = coord.z
	return new

def move_leg(robot, id, coord):
	'''
		Move a leg to a 3D coordinate
		Use inverse kinematics to set angles
		of the three motors of each leg independently
	'''
	newCoord = calc_leg(id, coord)
	angle = computeIK(newCoord.x, newCoord.y, newCoord.z)
	for m in robot.motors:
		if m.id == id*10+1:
			m.goal_position = angle[0]
		if m.id == id*10+2:
			m.goal_position = angle[1]
		if m.id == id*10+3:
			m.goal_position = angle[2]


class Robot(object):
	def __init__(self, config_file, height=-120, diameter=350):
		'''
			Create a simple class to interact with a robot using
			pypot library. This class allow functions to move and
			rotate the robot easier.

		'''
		self.robot = pypot.robot.from_json(config_file)

		for m in self.robot.motors:
			m.compliant = False

		self.height = height # height of the center of the robot
		self.diameter = diameter # diameter of robot leg's circle
		self.amplitude = 30 # amplitude/2 max of each steps in holonom walk
		self.step_height = 10 # maximum height of legs when walking
		self._base_points = [None]*6 # base points of legs when robot do nothing
		self._walk_points = [Point()]*6 # position of legs when robot walks
		self._rotate_points = [Point()]*6 # position of legs when robot rotate
		'''
			angles of legs on robot
						| |
					 ___|_|___       .
				 \ \|    4    |/ /  /|\
				  \_|3       5|_/    | Walk direction
				  / |2       6| \    |
				 / /|____1____|\ \   |
					    | |
					    | |
		'''
		self._angles = [270.0, 330.0, 30.0, 90.0, 150.0, 210.0]
		self._holonom_i = 0.0 # current index in holonom move
		self._rotate_i = 0.0 # current index in rotate move
		self.holonom_direction = 0.0
		self._rotate_angle = 0.0

		self._calculate_base_pos()

		self._is_walking = False # True if the robot walk
		self._is_rotating = False # True if the robot rotate
		self._is_two_legs = False
		self.center_point = Point()

	def __del__(self):
		print "robot deleted"
		for m in self.robot.motors:
			m.compliant = True

		#pypot.close(self.robot)

	def __repr__(self):
		return 'Robot'

	def _calculate_base_pos(self):
		'''
			Calculate the base position for the 6 legs arround the robot
			Base position is the position taken by the hexapod when he do nothing
		'''
		for i in range(len(self._angles)):
			self._base_points[i] = posOnCircle(self._angles[i], self.diameter, self.height)
			self._walk_points[i] = posOnCircle(self._angles[i], self.diameter, self.height)

	def base_pos(self):
		'''
			Put the robot in base position
		'''
		self._holonom_i = 0.0
		self._global_i = 0.0
		for i in range(len(self._angles)):
			move_leg(self.robot, i+1, self._base_points[i])

	def move_leg(self, id, pos):
		'''
			Move leg number ID at position pos
		'''

	def holonom_walk(self, speed, angle):
		'''
		Require robot translation when move function is called
		Move the robot at speed in angle direction
		'''
		self._holonom_i += speed
		self.holonom_direction = angle
		self._is_walking = True

	def _holonom_walk_calc(self):
		'''
			This function calculate position of each leg to move
			robot on required position.
			It can be move hexapod with two or tree groups of legs
		'''
		if self._is_walking:
			# if we want to move legs by groups of two
			if self._is_two_legs:
				pi_3 = math.pi * (2.0 / 3.0)

				tmp = rotateXY(Point(0, (math.cos(self._holonom_i) * self.amplitude), 0), self.holonom_direction)
				leg1 = step(self._holonom_i + (math.pi / 2))
				leg1.z *= self.step_height
				leg1.x = self._base_points[0].x + tmp.x
				leg1.y = self._base_points[0].y + tmp.y
				leg1.z += self._base_points[0].z

				leg4 = step(self._holonom_i + (math.pi / 2))
				leg4.z *= self.step_height
				leg4.x = self._base_points[3].x + tmp.x
				leg4.y = self._base_points[3].y + tmp.y
				leg4.z += self._base_points[3].z

				tmp = rotateXY(Point(0, (math.cos(self._holonom_i + pi_3) * self.amplitude), 0), self.holonom_direction)
				leg3 = step(self._holonom_i + ((math.pi / 2) + pi_3))
				leg3.z *= self.step_height
				leg3.x = self._base_points[2].x + tmp.x
				leg3.y = self._base_points[2].y + tmp.y
				leg3.z += self._base_points[2].z

				leg6 = step(self._holonom_i + ((math.pi / 2) + pi_3))
				leg6.z *= self.step_height
				leg6.x = self._base_points[5].x + tmp.x
				leg6.y = self._base_points[5].y + tmp.y
				leg6.z += self._base_points[5].z
				
				tmp = rotateXY(Point(0, (math.cos(self._holonom_i - pi_3) * self.amplitude), 0), self.holonom_direction)
				leg2 = step(self._holonom_i + ((math.pi / 2) - pi_3))
				leg2.z *= self.step_height
				leg2.x = self._base_points[1].x + tmp.x
				leg2.y = self._base_points[1].y + tmp.y
				leg2.z += self._base_points[1].z

				leg5 = step(self._holonom_i + ((math.pi / 2) - pi_3))
				leg5.z *= self.step_height
				leg5.x = self._base_points[4].x + tmp.x
				leg5.y = self._base_points[4].y + tmp.y
				leg5.z += self._base_points[4].z

			else: # if we want to move legs by groups of tree
				leg1 = step(self._holonom_i + (math.pi / 2))
				tmp = rotateXY(Point(0, (math.cos(self._holonom_i) * self.amplitude), 0), self.holonom_direction)
				leg1.z *= self.step_height
				leg1.x = self._base_points[0].x + tmp.x
				leg1.y = self._base_points[0].y + tmp.y
				leg1.z += self._base_points[0].z
				
				leg3 = step(self._holonom_i + (math.pi / 2))
				leg3.z *= self.step_height
				leg3.x = self._base_points[2].x + tmp.x
				leg3.y = self._base_points[2].y + tmp.y
				leg3.z += self._base_points[2].z

				leg5 = step(self._holonom_i + (math.pi / 2))
				leg5.z *= self.step_height
				leg5.x = self._base_points[4].x + tmp.x
				leg5.y = self._base_points[4].y + tmp.y
				leg5.z += self._base_points[4].z

				tmp = rotateXY(Point(0, -(math.cos(self._holonom_i) * self.amplitude), 0), self.holonom_direction)
				
				leg2 = step(self._holonom_i - (math.pi / 2))
				leg2.z *= self.step_height
				leg2.x = self._base_points[1].x + tmp.x
				leg2.y = self._base_points[1].y + tmp.y
				leg2.z += self._base_points[1].z

				leg4 = step(self._holonom_i - (math.pi / 2))
				leg4.z *= self.step_height
				leg4.x = self._base_points[3].x + tmp.x
				leg4.y = self._base_points[3].y + tmp.y
				leg4.z += self._base_points[3].z

				leg6 = step(self._holonom_i - (math.pi / 2))
				leg6.z *= self.step_height
				leg6.x = self._base_points[5].x + tmp.x
				leg6.y = self._base_points[5].y + tmp.y
				leg6.z += self._base_points[5].z

			self._base_points[0] = leg1
			self._base_points[1] = leg2
			self._base_points[2] = leg3
			self._base_points[3] = leg4
			self._base_points[4] = leg5
			self._base_points[5] = leg6




	def rotation(self, speed, angle):
		'''
			Require robot rotation when move function is called
			Rotate the robot at speed of angle
		'''
		self._rotate_i += speed
		self._rotate_angle = angle
		self._is_rotating = True
		

	def _rotation_calc(self):
		'''
			This function calculate the position of legs when
			a rotation is required.
		'''
		if self._is_rotating:
			rotPos1 = math.sin(self._rotate_i + math.pi) * self._rotate_angle
			rotPos2 = math.sin(self._rotate_i) * self._rotate_angle
			step1 = step(self._rotate_i)
			step2 = step(self._rotate_i + math.pi)

			if self._is_walking: # if robot walks, decelerate half legs and accelerate half others
				# in this case, angle is ignored because rotate only in relation to te walking direction
				'''for i in range(len(self._walk_points)):
					rotated_point = rotateXY(self._walk_points[i], self.holonom_direction)
					vector = Point(1, 1, 0) * (self._base_points[i] - self._walk_points[i])
					if (rotated_point.x >= 0):
						print "pattoune {} accelere, vecteur de pattoune {}".format(i+1, vector)
						self._walk_points[0] *= vector * 1.5
					else:
						print "pattoune {} ralentit, vecteur de pattoune {}".format(i+1, vector)
						self._walk_points[0] *= vector * 0.5'''
				self._base_points[0] = (self._base_points[0] + posOnCircle(self._angles[0] + rotPos2, self.diameter, self.height + (step1.z * self.step_height))) / 2.0
				self._base_points[1] = (self._base_points[1] + posOnCircle(self._angles[1] + rotPos1, self.diameter, self.height + (step2.z * self.step_height))) / 2.0
				self._base_points[2] = (self._base_points[2] + posOnCircle(self._angles[2] + rotPos2, self.diameter, self.height + (step1.z * self.step_height))) / 2.0
				self._base_points[3] = (self._base_points[3] + posOnCircle(self._angles[3] + rotPos1, self.diameter, self.height + (step2.z * self.step_height))) / 2.0
				self._base_points[4] = (self._base_points[4] + posOnCircle(self._angles[4] + rotPos2, self.diameter, self.height + (step1.z * self.step_height))) / 2.0
				self._base_points[5] = (self._base_points[5] + posOnCircle(self._angles[5] + rotPos1, self.diameter, self.height + (step2.z * self.step_height))) / 2.0

			else: # if robot don't walk, rotate on their own

				self._base_points[0] = posOnCircle(self._angles[0] + rotPos2, self.diameter, self.height + (step1.z * self.step_height))
				self._base_points[1] = posOnCircle(self._angles[1] + rotPos1, self.diameter, self.height + (step2.z * self.step_height))
				self._base_points[2] = posOnCircle(self._angles[2] + rotPos2, self.diameter, self.height + (step1.z * self.step_height))
				self._base_points[3] = posOnCircle(self._angles[3] + rotPos1, self.diameter, self.height + (step2.z * self.step_height))
				self._base_points[4] = posOnCircle(self._angles[4] + rotPos2, self.diameter, self.height + (step1.z * self.step_height))
				self._base_points[5] = posOnCircle(self._angles[5] + rotPos1, self.diameter, self.height + (step2.z * self.step_height))


	def move(self):
		'''
			This function update position of all legs calling 
		'''
		self._calculate_base_pos()
		self._rotation_calc()
		self._holonom_walk_calc()
		for i in range(len(self._angles)):
			new_pos = self.center_point + self._base_points[i]
			move_leg(self.robot, i+1, new_pos)
		
		# reset booleans of movment
		if self._is_walking:
			self._rotate_i = 0.0
		if self._is_rotating:
			self._holonom_i = 0.0

		self._is_walking = False
		self._is_rotating = False
