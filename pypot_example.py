import itertools
import time
import numpy
import pypot.dynamixel
import pypot.dynamixel.motor
import math

L1 = 51.0 #mm
L2 = 63.7 #mm
L3 = 93.0 #mm
ALPHA = math.radians(20.69)
BETA = math.radians(5.06)

def leg_dk(theta1, theta2, theta3, origin = {"x":0, "y":0, "z":0}, l1=L1, l2=L2, l3=L3, alpha=ALPHA, beta=BETA):
    theta2 = -theta2
    theta2 += -alpha # correction of the theta 2 angle
    theta3 += alpha + beta - math.radians(90) # correction of the theta 3 angle
    
    
    ## I think it's good to consider the length of the arms with cosine because they are'nt aligned
    #l2 = math.cos(alpha) * l2 # correction of the L2 length
    #l3 = math.cos(beta) * l3 # correction of the L3 length
    p1 = {
	"x": origin["x"] + l1 * math.cos(theta1),
	"y": origin["y"] + l1 * math.sin(theta1),
	"z": origin["z"]
    }

    d12 = l2 * math.cos(theta2)

    p2 = {
        "x": p1["x"] + math.cos(theta1) * l2 * math.cos(theta2),
        "y": p1["y"] + math.sin(theta1) * l2 * math.cos(theta2),
		"z": p1["z"] + l2 * math.sin(theta2)
    }
    
    d23 = l3 * math.cos(theta2 + theta3)
    
    p3 = {
        "x": (l1 + d12 + d23) * math.cos(theta1),
        "y": (l1 + d12 + d23) * math.sin(theta1),
        "z": p2["z"] + l3 * math.sin(theta2 + theta3)
    }
    
    return p3

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

if __name__ == '__main__':

    # we first open the Dynamixel serial port
    with pypot.dynamixel.io.io.DxlIO('/dev/ttyUSB0', baudrate=1000000) as dxl_io:
	
        # we can scan the motors
        #found_ids = dxl_io.scan()  # this may take several seconds
	found_ids = [10,11,12]        
	print 'Detected:', found_ids
	dxl_io.enable_torque(found_ids)
	#print 'Limites : ' , dxl_io.get_angle_limit(found_ids)
        # we power on the motors
	#dxl_io.change_id({found_ids[0]:10,found_ids[1]:11,found_ids[2]:12})

        # we get the current positions
        print 'Current pos:', dxl_io.get_present_position(found_ids)

	angles = [90,0,0]
	print 'Direction', angles
        # we create a python dictionnary: {id0 : position0, id1 : position1...}
        pos = dict(zip(found_ids, angles))
        #print 'Cmd:', pos

        # we send these new positions
        dxl_io.set_goal_position(pos)
        time.sleep(1)  # we wait for 1s
	
        # we get the current positions
	coord = leg_dk(math.radians(90),math.radians(0),math.radians(-0))
	actualPos = dxl_io.get_present_position(found_ids)
	newcoord = leg_dk(math.radians(actualPos[0]),math.radians(actualPos[1]),math.radians(actualPos[2]))
        print 'New pos:', dxl_io.get_present_position(found_ids)
	print 'What we should find' , leg_ik(coord['x'],coord['y'],coord['z'])
	print 'What we find' , leg_ik(newcoord['x'],newcoord['y'],newcoord['z'])
        # we power off the motors
	
	dxl_io.disable_torque(found_ids)
        time.sleep(1)  # we wait for 1s
