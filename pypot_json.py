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

def base_pos(robot):
	for m in robot.motors:			
			m.compliant = False
			m.goal_position = 0
	robot.motor_21.goal_position = -45
	robot.motor_31.goal_position = 45
	robot.motor_51.goal_position = -45
	robot.motor_61.goal_position = 45
	time.sleep(1)



if __name__ == '__main__':

	with closing(pypot.robot.from_json('robotConfig2.json')) as robot:
		#Allow motors to move and initialize them
		'''
		for m in robot.motors:			
			m.compliant = False
			m.goal_position = 0
		'''
		base_pos(robot)
		for m in robot.duplet:
			for n in m.shoulders:
				m.goal_position = 15
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


