from Point import *

import math


# CONSTANT DEFINITIONS

L1 = 51.0 #mm
L2 = 63.7 #mm
L3 = 93.0 #mm
ALPHA = math.radians(-20.69)
BETA = math.radians(5.06)


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
		math.cos(theta) * (diameter/2.0),
		math.sin(theta) * (diameter/2.0),
		height
	)

class Robot(object):
	def __init__(self):
		''''''
	def __repr__(self):
		return 'Robot'

	def base_pos(self):
		'''
			Put the robot in base position
		'''

	def move_leg(self, id, pos):
		'''
			Move leg n°ID at position pos
		'''
