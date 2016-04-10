import itertools
import time
import numpy
import pypot.dynamixel
import pypot.dynamixel.motor
import pypot.robot
from contextlib import closing
import math
import pprint #Var dump
L1 = 51.0 #mm
L2 = 63.7 #mm
L3 = 93.0 #mm
ALPHA = math.radians(-20.69)
BETA = math.radians(5.06)

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
	print math.degrees(theta1)
	print math.degrees(theta2C)
	print math.degrees(theta3C)
    return [math.degrees(theta1), math.degrees(theta2C), math.degrees(theta3C)]

def base_pos(robot):
	for m in robot.shoulders:			
			m.compliant = False
			m.goal_position = 20
	
	robot.motor_11.goal_position = -15
	
	robot.motor_21.goal_position = -15
	
	robot.motor_31.goal_position = -15

	robot.motor_41.goal_position = 15

	robot.motor_51.goal_position = -15

	robot.motor_61.goal_position = -15
	time.sleep(2)

def move_leg(robot, id, coord):
	print coord
	angle = computeIK(coord["x"], coord["y"], coord["z"])
	print angle
	for m in robot.motors:
		if m.id == id*10+1:
			m.goal_position = angle[0]
			print "Jambe 11"
			print m.id
		if m.id == id*10+2:
			m.goal_position = angle[1]
			print "Jambe 12"
			print m.id
		if m.id == id*10+3:
			m.goal_position = angle[2]
			print "Jambe 13"
			print m.id
	'''
	robot.motors['motor_'+str(id)+'1'].goal_position = angle[0];
	robot.motors['motor_'+str(id)+'2'].goal_position = angle[1];
	robot.motors['motor_'+str(id)+'3'].goal_position = angle[2];
	'''

def step(i):
	coord = {
        "x": 0,
        "y": 0,
        "z": 40 * math.log(max(1, 1 + math.cos(i)*100),100)
    	} 
	return coord

if __name__ == '__main__':

	with closing(pypot.robot.from_json('robotConfig2.json')) as robot:
		#Allow motors to move and initialize them
		
		for m in robot.motors:			
			m.compliant = False
			m.goal_position = 0
		time.sleep(0.5)
		
		coord1 = {
        "x": 150,
        "y": 0,
        "z": 0
    	}
		coord2 = {
        "x": 150,
        "y": 0,
        "z": 0
    	}
		
		i = 0
		while True:
			'''
			coord1["z"] = 100*math.log(max(1, 1 + math.cos(i)*100),100) - 50
			coord1["y"] = math.sin(i)* 100

			coord2["z"] = 100*math.log(max(1, 1 + math.cos(i+math.pi)*100),100) - 50
			coord2["y"] = math.sin(i)* 100
			'''
			leg1 = step(i + math.pi)
			leg1["z"] = -leg1["z"]
			leg1["x"] += 100
			leg1["y"] = math.cos(i - (math.pi / 2)) * 50
			leg1["z"] += -100
			leg3 = step(i + math.pi)
			leg3["z"] = -leg3["z"]
			leg3["y"] += 40
			leg3["x"] = math.cos(i + (math.pi / 2)) * 50 + 100
			leg3["z"] += -100
			leg5 = step(i + math.pi)
			leg5["z"] = -leg5["z"]
			leg5["y"] += -40
			leg5["x"] = math.cos(i - (math.pi / 2)) * 50 + 100
			leg5["z"] += -100

			leg2 = step(i + math.pi)
			leg2["z"] = -leg5["z"]
			leg2["y"] += -40
			leg2["x"] = math.cos(i - (math.pi / 2)) * 50 + 100
			leg2["z"] += -100
			leg4 = step(i + math.pi)
			leg4["z"] = -leg1["z"]
			leg4["x"] += 100
			leg4["y"] = math.cos(i - (math.pi / 2)) * 50
			leg4["z"] += -100
			leg6 = step(i + math.pi)
			leg6["z"] = -leg1["z"]
			leg6["x"] += 100
			leg6["y"] = math.cos(i - (math.pi / 2)) * 50
			leg6["z"] += -100

			#move_leg(robot, 1, leg1)
			move_leg(robot, 2, leg1)
			#move_leg(robot, 3, leg3)
			move_leg(robot, 4, leg1)
			#move_leg(robot, 5, leg5)
			move_leg(robot, 6, leg1)

			i+=0.1
			time.sleep(0.05)
		
		
		robot.motor_21.goal_position = -45
		
		robot.motor_31.goal_position = 45
		
		
		robot.motor_51.goal_position = -45

		robot.motor_61.goal_position = 45
		
		time.sleep(0.5)
	
		#base_pos(robot)
		'''
		for m in robot.duplet:
			for n in m.shoulders:
				m.goal_position = 15
		'''
		'''
		sign = 1
		while True:
			sign = sign * -1
			for m in robot.shoulders:
				m.compliant = False
				m.goal_position = 10*sign
			time.sleep(0.5)
		'''
		'''
		time.sleep(0.5)
		for m in robot.leg1:
			m.goal_position=20
		time.sleep(0.5)
		for m in robot.leg2:
			m.goal_position=20
		time.sleep(0.5)
		for m in robot.leg3:
			m.goal_position=20
		time.sleep(0.5)
		for m in robot.leg4:
			m.goal_position=20
		time.sleep(0.5)
		for m in robot.leg5:
			m.goal_position=20
		time.sleep(0.5)
		for m in robot.leg6:
			m.goal_position=20
		time.sleep(0.5)

		for m in robot.leg1:
			m.goal_position=0
		time.sleep(0.5)
		for m in robot.leg2:
			m.goal_position=0
		time.sleep(0.5)
		for m in robot.leg3:
			m.goal_position=0
		time.sleep(0.5)
		for m in robot.leg4:
			m.goal_position=0
		time.sleep(0.5)
		for m in robot.leg5:
			m.goal_position=0
		time.sleep(0.5)
		for m in robot.leg6:
			m.goal_position=0
		time.sleep(0.5)
		'''
		#Forbid motors to move
		for m in robot.motors:			
			m.compliant = True


