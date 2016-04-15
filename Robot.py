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

def rotateXY(x, y, z, angle):
	'''
		Rotate the point at position pos arround the 0,0,0
		point with angle in degrees. Rotation arround Z axis.
	'''
	pos = Point(x, y, z)
	theta = math.radians(angle)
	return pos * Point(
		math.cos(theta) + -math.sin(theta),
		math.sin(theta) + math.cos(theta),
		0
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
	return Point(
		0,
		0,
		math.log(max(1, 1 + math.cos(i)*10),10)
	)

def calc_leg(id, coord):
	'''
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
		''''''
		self.robot = pypot.robot.from_json(config_file)

		for m in self.robot.motors:
			m.compliant = False

		self.height = height
		self.diameter = diameter
		self.amplitude = 30
		self.step_height = 10
		self._base_points = [None]*6
		self._walk_points = [None]*6
		self._angles = [270.0, 330.0, 30.0, 90.0, 150.0, 210.0]
		self._calculate_base_pos()
		self.sine_i = 0.0
		self.holonom_i = 0.0
		self.center_point = Point()

	def __del__(self):
		for m in self.robot.motors:
			m.compliant = True

		#pypot.close(self.robot)

	def __repr__(self):
		return 'Robot'

	def _calculate_base_pos(self):
		'''
		'''
		for i in range(len(self._angles)):
			self._base_points[i] = posOnCircle(self._angles[i], self.diameter, self.height)

	def base_pos(self):
		'''
			Put the robot in base position
		'''
		self.sine_i = 0.0
		self.holonom_i = 0.0
		for i in range(len(self._angles)):
			move_leg(self.robot, i+1, self._base_points[i])

	def move_leg(self, id, pos):
		'''
			Move leg number ID at position pos
		'''

	def holonom_walk(self, i, speed, angle):
		self.holonom_i += (i * speed)
		leg1 = step(self.holonom_i + (math.pi / 2))
		tmp = rotateXY(0, (math.cos(self.holonom_i) * self.amplitude), 0, angle)
		leg1.z *= self.step_height
		leg1.x = self._base_points[0].x + tmp.x
		leg1.y = self._base_points[0].y + tmp.y
		leg1.z += self._base_points[0].z
		
		leg3 = step(self.holonom_i + (math.pi / 2))
		leg3.z *= self.step_height
		leg3.x = self._base_points[2].x + tmp.x
		leg3.y = self._base_points[2].y + tmp.y
		leg3.z += self._base_points[2].z

		leg5 = step(self.holonom_i + (math.pi / 2))
		leg5.z *= self.step_height
		leg5.x = self._base_points[4].x + tmp.x
		leg5.y = self._base_points[4].y + tmp.y
		leg5.z += self._base_points[4].z

		tmp = rotateXY(0, -(math.cos(self.holonom_i) * self.amplitude), 0, angle)
		
		leg2 = step(self.holonom_i - (math.pi / 2))
		leg2.z *= self.step_height
		leg2.x = self._base_points[1].x + tmp.x
		leg2.y = self._base_points[1].y + tmp.y
		leg2.z += self._base_points[1].z

		leg4 = step(self.holonom_i - (math.pi / 2))
		leg4.z *= self.step_height
		leg4.x = self._base_points[3].x + tmp.x
		leg4.y = self._base_points[3].y + tmp.y
		leg4.z += self._base_points[3].z

		leg6 = step(self.holonom_i - (math.pi / 2))
		leg6.z *= self.step_height
		leg6.x = self._base_points[5].x + tmp.x
		leg6.y = self._base_points[5].y + tmp.y
		leg6.z += self._base_points[5].z

		self._walk_points[0] = leg1
		self._walk_points[1] = leg2
		self._walk_points[2] = leg3
		self._walk_points[3] = leg4
		self._walk_points[4] = leg5
		self._walk_points[5] = leg6

	def move(self):
		'''
		'''
		for i in range(len(self._angles)):
			move_leg(self.robot, i+1, self._base_points[i] + self._walk_points[i] + self.center_point)
