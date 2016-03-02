import itertools
import time
import numpy
import pypot.dynamixel
import pypot.dynamixel.motor
import pypot.robot
import math

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

def move_leg(dxl_io, ids, x, y, z):
	angles = leg_ik(x,y,z)
	print 'Direction', angles
        # we create a python dictionnary: {id0 : position0, id1 : position1...}
        pos = dict(zip(ids, angles))
        #print 'Cmd:', pos

        # we send these new positions
        dxl_io.set_goal_position(pos)
        time.sleep(1)  # we wait for 1s

if __name__ == '__main__':

    # we first open the Dynamixel serial port
    with pypot.dynamixel.io.io.DxlIO('/dev/ttyUSB0', baudrate=1000000) as dxl_io:
	
        # we can scan the motors
        #found_ids = dxl_io.scan()  # this may take several seconds
	found_ids = [11,12,13,21,22,23,31,32,33,41,42,43,51,52,53,61,62,63]        
	print 'Detected:', found_ids
	#dxl_io.enable_torque(found_ids)
	#print 'Limites : ' , dxl_io.get_angle_limit(found_ids)
        # we power on the motors
	
	#dxl_io.set_goal_position({63:20})
	time.sleep(1)
        # we get the current positions
        #print 'Current pos:', dxl_io.get_present_position(found_ids)
	
	pos = dict(zip(found_ids,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]))
	dxl_io.set_goal_position(pos)
	time.sleep(1)
	
	#move_leg(dxl_io, found_ids, 150, 0, 0)
	'''
	while True:
		move_leg(dxl_io, found_ids, 170, -45, 35)
		move_leg(dxl_io, found_ids, 170, -45, 0)
		move_leg(dxl_io, found_ids, 170, -45, 35)
		move_leg(dxl_io, found_ids, 170, 0, 35)
		move_leg(dxl_io, found_ids, 170, 0, 0)
		move_leg(dxl_io, found_ids, 170, 0, 35)
		move_leg(dxl_io, found_ids, 170, 45, 35)
		move_leg(dxl_io, found_ids, 170, 45, 0)
		move_leg(dxl_io, found_ids, 170, 45, 35)
	'''

        # we get the current positions
        # we power off the motors
	dxl_io.disable_torque(found_ids)
        #time.sleep(1)  # we wait for 1s
