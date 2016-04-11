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

def calc_leg(id, coord):
	newX = coord["x"]
	newY = coord["y"]
	newZ = coord["z"]	
	if id==1:
		newX = -coord["y"] - 102
		newY = coord["x"]
	elif id==2:
		newX = coord["x"]
		newY = coord["y"]
		newY = newY + 24
		newX = newX - 74 
	elif id==3:
		newX = coord["x"]
		newY = coord["y"]
		newY = newY - 24
		newX = newX - 74
	elif id==4:
		newX = coord["y"] - 102
		newY = -coord["x"]
	elif id==5:
		newX = -coord["x"]
		newY = -coord["y"]
		newY = newY + 24
		newX = newX - 74
	elif id==6:
		newX = -coord["x"]
		newY = -coord["y"]
		newY = newY - 24
		newX = newX - 74
	return {"x":newX, "y":newY, "z":coord["z"]}

def move_leg(robot, id, coord):
	newCoord = calc_leg(id, coord)
	angle = computeIK(newCoord["x"], newCoord["y"], newCoord["z"])
	for m in robot.motors:
		if m.id == id*10+1:
			m.goal_position = angle[0]
		if m.id == id*10+2:
			m.goal_position = angle[1]
		if m.id == id*10+3:
			m.goal_position = angle[2]

def step(i):
	coord = {
        "x": 0,
        "y": 0,
        "z": math.log(max(1, 1 + math.cos(i)*10),10)
    	} 
	return coord

def rotateXY(x, y, z, theta):
	theta = math.radians(theta)
	return {"y": (math.cos(theta) * x) +
		(math.sin(theta) * y),
		"x": (-math.sin(theta) * x) +
		(math.cos(theta)) * y,
		"z": z}

if __name__ == '__main__':

	with closing(pypot.robot.from_json('robotConfig2.json')) as robot:
		#Allow motors to move and initialize them
		coord = {
					"x": 0,
					"y": 0,
					"z": -50
		}
		for m in robot.motors:			
			m.compliant = False
			#m.goal_position = 0
		time.sleep(0.1)

		move_leg(robot, 1, {"x":0, "y":-200, "z":-120})	
		move_leg(robot, 2, {"x":150, "y":-80, "z":-120})
		move_leg(robot, 3, {"x":150, "y":80, "z":-120})
		move_leg(robot, 4, {"x":0, "y":200, "z":-120})
		move_leg(robot, 5, {"x":-150, "y":80, "z":-120})
		move_leg(robot, 6, {"x":-150, "y":-80, "z":-120})

		time.sleep(3)

		print rotateXY(0, 0, 0, 30)
		
		i = 0
		cpt = 0
		height = 120
		amplitude = 30
		stepHeight = 40
		angle = 30
		while True:
			leg1 = step(i + (math.pi / 2))
			tmp = rotateXY(0, (math.cos(i) * amplitude), 0, angle)
			print tmp
			leg1["z"] *= stepHeight
			leg1["x"] = 0 + tmp["x"]
			leg1["y"] = -200 + tmp["y"]
			leg1["z"] += -height
			
			leg3 = step(i + (math.pi / 2))
			leg3["z"] *= stepHeight
			leg3["x"] = 150 + tmp["x"]
			leg3["y"] = 80 + tmp["y"]
			leg3["z"] += -height

			leg5 = step(i + (math.pi / 2))
			leg5["z"] *= stepHeight
			leg5["x"] = -150 + tmp["x"]
			leg5["y"] = 80 + tmp["y"]
			leg5["z"] += -height

			tmp = rotateXY(0, -(math.cos(i) * amplitude), 0, angle)
			
			leg4 = step(i - (math.pi / 2))
			leg4["z"] *= stepHeight
			leg4["x"] = 0 + tmp["x"]
			leg4["y"] = 200 + tmp["y"]
			leg4["z"] += -height

			leg2 = step(i - (math.pi / 2))
			leg2["z"] *= stepHeight
			leg2["x"] = 150 + tmp["x"]
			leg2["y"] = -80 + tmp["y"]
			leg2["z"] += -height

			leg6 = step(i - (math.pi / 2))
			leg6["z"] *= stepHeight
			leg6["x"] = -150 + tmp["x"]
			leg6["y"] = -80 + tmp["y"]
			leg6["z"] += -height	

			move_leg(robot, 1, leg1)
			move_leg(robot, 2, leg2)
			move_leg(robot, 3, leg3)
			move_leg(robot, 4, leg4)
			move_leg(robot, 5, leg5)
			move_leg(robot, 6, leg6)

			i+=0.2
			cpt+=1
			if cpt%75 ==0:
				angle+=60
			time.sleep(0.05)
		
		#Forbid motors to move
		for m in robot.motors:			
			m.compliant = True


