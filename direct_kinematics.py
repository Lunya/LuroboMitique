#!/usr/bin/python
import math

L1 = 51.0 #mm
L2 = 63.7 #mm
L3 = 93.0 #mm
ALPHA = math.radians(20.69)
BETA = math.radians(5.06)

# the function takes angles in radians
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

# For testing purposes only.
def approximation(value1, value2, error):
    if value1>value2-error and value1<value2+error:
        return True
    else:
        return False
        
def test():
    error = 1 #mm
    
    #Test 1
    resultToFind = {"x":118.79,"y":0.0,"z":-115.14}
    resultFound = leg_dk(math.radians(0), math.radians(0), math.radians(0))
    print("x:{x:.2f} y:{y:.2f} z:{z:.2f}".format(**resultFound))
    if approximation(resultToFind['x'],resultFound['x'],error) and approximation(resultToFind['y'], resultFound['y'],error) and approximation(resultToFind['z'], resultFound['z'], error):
        print("Validé")
    
    #Test 2
    resultToFind = {"x":0.0,"y":118.79,"z":-115.14}
    resultFound = leg_dk(math.radians(90), math.radians(0), math.radians(0))
    print("x:{x:.2f} y:{y:.2f} z:{z:.2f}".format(**resultFound))
    if approximation(resultToFind['x'],resultFound['x'],error) and approximation(resultToFind['y'], resultFound['y'],error) and approximation(resultToFind['z'], resultFound['z'], error):
        print("Validé")
        
    #Test 3
    resultToFind = {"x":-64.14,"y":0.0,"z":-67.79}
    resultFound = leg_dk(math.radians(180), math.radians(-30.501), math.radians(-67.819))
    print("x:{x:.2f} y:{y:.2f} z:{z:.2f}".format(**resultFound))
    if approximation(resultToFind['x'],resultFound['x'],error) and approximation(resultToFind['y'], resultFound['y'],error) and approximation(resultToFind['z'], resultFound['z'], error):
        print("Validé")
        
    #Test 4
    resultToFind = {"x":203.23,"y":0.0,"z":-14.30}
    resultFound = leg_dk(math.radians(0), math.radians(-30.645), math.radians(38.501))
    print("x:{x:.2f} y:{y:.2f} z:{z:.2f}".format(**resultFound))
    if approximation(resultToFind['x'],resultFound['x'],error) and approximation(resultToFind['y'], resultFound['y'],error) and approximation(resultToFind['z'], resultFound['z'], error):
        print("Validé")
    
    #Test 5   
    resultToFind = {"x":213.23,"y":10.0,"z":-4.30}
    resultFound = leg_dk(math.radians(0), math.radians(-30.645), math.radians(38.501), {"x":10, "y":10, "z":10})
    print("x:{x:.2f} y:{y:.2f} z:{z:.2f}".format(**resultFound))
    if approximation(resultToFind['x'],resultFound['x'],error) and approximation(resultToFind['y'], resultFound['y'],error) and approximation(resultToFind['z'], resultFound['z'], error):
        print("Validé")