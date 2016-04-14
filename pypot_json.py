import itertools
import time
import numpy
import pypot.dynamixel
import pypot.dynamixel.motor
import pypot.robot
from contextlib import closing
import math
import pygame

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

def posOnCircle(angle, diameter, height):
	theta = math.radians(angle)
	return {"x": math.cos(theta) * diameter/2,
		"y": math.sin(theta) * diameter/2,
		"z": height}

def rotationLeg(angle, rotationAngle, diameter, height):
	return posOnCircle(angle+rotationAngle, diameter, height)
	
def robotRotation(angles, rotationAngle, diameter, height):
	newBasePoints = []
	for i in range(6):
		if i==1 or i==3 or i==5:
			newBasePoints.append(rotationLeg(angles[i], rotationAngle, diameter, height))
		#else:
		#	newBasePoints.append(rotationLeg(angles[i], rotationAngle, diameter, height))

	return newBasePoints

if __name__ == '__main__':

	with closing(pypot.robot.from_json('robotConfig3.json')) as robot:
		#Allow motors to move and initialize them
		coord = {
					"x": 0,
					"y": 0,
					"z": -50
		}
		for m in robot.motors:			
			m.compliant = False
			m.goal_position = 0
		time.sleep(0.1)
		

		height = 120
		amplitude = 30
		stepHeight = 10
		#diameter = 200 + amplitude
		diameter = 350
		height = -120
		angles = []
		angles.append(270)
		angles.append(330)
		angles.append(30)
		angles.append(90)
		angles.append(150)
		angles.append(210)
		basePoints = []
		basePoints.append(posOnCircle(angles[0], diameter, height))
		basePoints.append(posOnCircle(angles[1], diameter, height))
		basePoints.append(posOnCircle(angles[2], diameter, height))
		basePoints.append(posOnCircle(angles[3], diameter, height))
		basePoints.append(posOnCircle(angles[4], diameter, height))
		basePoints.append(posOnCircle(angles[5], diameter, height))
		print basePoints[0]
		print basePoints[1]
		print basePoints[2]
		print basePoints[3]
		print basePoints[4]
		print basePoints[5]
		
		move_leg(robot, 1, basePoints[0])
		move_leg(robot, 2, basePoints[1])
		move_leg(robot, 3, basePoints[2])
		move_leg(robot, 4, basePoints[3])
		move_leg(robot, 5, basePoints[4])
		move_leg(robot, 6, basePoints[5])
		
		time.sleep(3)

		#########################    PYGAME    ####################################

		background_color = (220,220,220)
		button_color = (80,80,80)

		size_action_surface = 200
		rectangle_action = (100, 100, size_action_surface, size_action_surface)
		rectangle_pointer = (0, 0, 10, 10)

		diameterMove = 300
		amplitudeMove = 150
		offsetX = 0
		offsetY = 0
		centerMove = []
		centerMove.append(posOnCircle(angles[0], diameterMove, height))
		centerMove.append(posOnCircle(angles[1], diameterMove, height))
		centerMove.append(posOnCircle(angles[2], diameterMove, height))
		centerMove.append(posOnCircle(angles[3], diameterMove, height))
		centerMove.append(posOnCircle(angles[4], diameterMove, height))
		centerMove.append(posOnCircle(angles[5], diameterMove, height))

		pygame.init()
		running = True
		screen = pygame.display.set_mode((600, 400))
		pygame.display.set_caption("LuroboMitique")
		clock = pygame.time.Clock()

		while running:
			clock.tick(120) # min FPS

			leg1 = centerMove[0].copy()
			leg2 = centerMove[1].copy()
			leg3 = centerMove[2].copy()
			leg4 = centerMove[3].copy()
			leg5 = centerMove[4].copy()
			leg6 = centerMove[5].copy()

			for event in pygame.event.get():
				if event.type  == pygame.QUIT:
					running = False
				elif event.type == pygame.MOUSEMOTION:
					if (event.pos[0] >= rectangle_action[0]) and\
					(event.pos[0] <= (rectangle_action[0] + rectangle_action[2])) and\
					(event.pos[1] >= rectangle_action[1]) and\
					(event.pos[1] <= (rectangle_action[1] + rectangle_action[3])):
						rectangle_pointer = (event.pos[0]-5, event.pos[1]-5, 10, 10)
						offsetX = (((event.pos[0] - rectangle_action[0]) - (size_action_surface / 2)) * amplitudeMove) / size_action_surface
						offsetY = (((event.pos[1] - rectangle_action[1]) - (size_action_surface / 2)) * amplitudeMove) / size_action_surface
			


			leg1["x"] += offsetX
			leg1["y"] += offsetY
			leg2["x"] += offsetX
			leg2["y"] += offsetY
			leg3["x"] += offsetX
			leg3["y"] += offsetY
			leg4["x"] += offsetX
			leg4["y"] += offsetY
			leg5["x"] += offsetX
			leg5["y"] += offsetY
			leg6["x"] += offsetX
			leg6["y"] += offsetY


			move_leg(robot, 1, leg1)
			move_leg(robot, 2, leg2)
			move_leg(robot, 3, leg3)
			move_leg(robot, 4, leg4)
			move_leg(robot, 5, leg5)
			move_leg(robot, 6, leg6)
			

			screen.fill(background_color)
			pygame.draw.rect(screen, button_color, rectangle_action, 4)
			pygame.draw.rect(screen, (255, 0, 0), rectangle_pointer, 4)
			pygame.display.flip()

		pygame.quit()
		#########################    END PYGAME    ####################################
		
		print "ROTATION"
		"""basePoints = robotRotation(angles , 0, diameter, height)
		move_leg(robot, 1, basePoints[0])
		#move_leg(robot, 2, basePoints[1])
		move_leg(robot, 3, basePoints[2])
		#move_leg(robot, 4, basePoints[3])
		move_leg(robot, 5, basePoints[4])
		#move_leg(robot, 6, basePoints[5])
		time.sleep(2)
		#move_leg(robot, 1, basePoints[0])
		move_leg(robot, 2, basePoints[1])
		#move_leg(robot, 3, basePoints[2])
		move_leg(robot, 4, basePoints[3])
		#move_leg(robot, 5, basePoints[4])
		move_leg(robot, 6, basePoints[5])
		for i in range(6):
			angles[i] += 5		
		time.sleep(3)
		"""
		'''
		basePoints = robotRotation(angles , 45, diameter, height)
		move_leg(robot, 1, basePoints[0])
		move_leg(robot, 2, basePoints[1])
		move_leg(robot, 3, basePoints[2])
		move_leg(robot, 4, basePoints[3])
		move_leg(robot, 5, basePoints[4])
		move_leg(robot, 6, basePoints[5])
		for i in range(6):
			angles[i] += 45		
		time.sleep(3)		
		'''
		i = 0
		cpt = 0
		angle = 5
		
		while True:
			# de la rotation
			rotPos1 = math.sin(i + math.pi) * angle
			rotPos2 = math.sin(i) * angle
			step1 = step(i + math.pi)
			step1["z"] *= stepHeight
			step2 = step(i)
			step2["z"] *= stepHeight
			
			leg1 = posOnCircle(angles[0] + rotPos2, diameter, height)
			leg1["z"] += step1["z"]
			leg2 = posOnCircle(angles[1] + rotPos1, diameter, height)
			leg2["z"] += step2["z"]
			leg3 = posOnCircle(angles[2] + rotPos2, diameter, height)
			leg3["z"] += step1["z"]
			leg4 = posOnCircle(angles[3] + rotPos1, diameter, height)
			leg4["z"] += step2["z"]
			leg5 = posOnCircle(angles[4] + rotPos2, diameter, height)
			leg5["z"] += step1["z"]
			leg6 = posOnCircle(angles[5] + rotPos1, diameter, height)
			leg6["z"] += step2["z"]

			move_leg(robot, 1, leg1)
			move_leg(robot, 2, leg2)
			move_leg(robot, 3, leg3)
			move_leg(robot, 4, leg4)
			move_leg(robot, 5, leg5)
			move_leg(robot, 6, leg6)

			i+=0.1
			cpt+=1
			time.sleep(0.02)

		
		i = 0
		cpt = 0
		angle = 5
		while True:
			leg1 = step(i + (math.pi / 2))
			tmp = rotateXY(0, (math.cos(i) * amplitude), 0, angle)
			leg1["z"] *= stepHeight
			leg1["x"] = basePoints[0]["x"] + tmp["x"]
			leg1["y"] = basePoints[0]["y"] + tmp["y"]
			leg1["z"] += basePoints[0]["z"]
			
			leg3 = step(i + (math.pi / 2))
			leg3["z"] *= stepHeight
			leg3["x"] = basePoints[2]["x"] + tmp["x"]
			leg3["y"] = basePoints[2]["y"] + tmp["y"]
			leg3["z"] += basePoints[2]["z"]

			leg5 = step(i + (math.pi / 2))
			leg5["z"] *= stepHeight
			leg5["x"] = basePoints[4]["x"] + tmp["x"]
			leg5["y"] = basePoints[4]["y"] + tmp["y"]
			leg5["z"] += basePoints[4]["z"]

			tmp = rotateXY(0, -(math.cos(i) * amplitude), 0, angle)
			
			leg2 = step(i - (math.pi / 2))
			leg2["z"] *= stepHeight
			leg2["x"] = basePoints[1]["x"] + tmp["x"]
			leg2["y"] = basePoints[1]["y"] + tmp["y"]
			leg2["z"] += basePoints[1]["z"]

			leg4 = step(i - (math.pi / 2))
			leg4["z"] *= stepHeight
			leg4["x"] = basePoints[3]["x"] + tmp["x"]
			leg4["y"] = basePoints[3]["y"] + tmp["y"]
			leg4["z"] += basePoints[3]["z"]

			leg6 = step(i - (math.pi / 2))
			leg6["z"] *= stepHeight
			leg6["x"] = basePoints[5]["x"] + tmp["x"]
			leg6["y"] = basePoints[5]["y"] + tmp["y"]
			leg6["z"] += basePoints[5]["z"]
			
			move_leg(robot, 1, leg1)
			move_leg(robot, 2, leg2)
			move_leg(robot, 3, leg3)
			move_leg(robot, 4, leg4)
			move_leg(robot, 5, leg5)
			move_leg(robot, 6, leg6)
			
			i+=0.1
			cpt+=1
			if cpt%75 ==0:
				angle+=60
			time.sleep(0.02)
		
		#Forbid motors to move
		for m in robot.motors:			
			m.compliant = True


