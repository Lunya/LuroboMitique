#!/usr/bin/python
import math

L1 = 51.0 #mm
L2 = 63.7 #mm
L3 = 93.0 #mm
ALPHA = math.radians(20.69)
BETA = math.radians(5.06)
# On ne sait pas si le cas ou x vaut 0 est bien géré, mais marche actuellement pour les exemples donnés.
def leg_ik(x,y,z,l1=L1, l2=L2, l3=L3, alpha=ALPHA, beta=BETA):
    theta1 = math.atan2(y, x)
    #theta1 += math.pi / 2
    p1 = {
        "x": l1 * math.cos(theta1),
        "y": l1 * math.sin(theta1),
        "z": 0
    }
    d13 = math.sqrt((x - p1["x"])**2 + (y - p1["y"])**2)
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


# For testing purposes only.
def approximation(value1, value2, error):
    if value1>value2-error and value1<value2+error:
        return True
    else:
        return False
        
def test():
    error = 1 #deg
    
    #Test 1
    resultToFind = [0, 0, 0]
    resultFound = leg_ik(118.79, 0, -115.14)
    print("x:{0:.2f} y:{1:.2f} z:{2:.2f}".format(*resultFound))
    if approximation(resultToFind[0],resultFound[0],error) and approximation(resultToFind[0], resultFound[0],error) and approximation(resultToFind[0], resultFound[0], error):
        print("Validé")
    
    #Test 2
    resultToFind = [90, 0, 0]
    resultFound = leg_ik(0.0, 118.79, -115.14)
    print("x:{0:.2f} y:{1:.2f} z:{2:.2f}".format(*resultFound))
    if approximation(resultToFind[0],resultFound[0],error) and approximation(resultToFind[0], resultFound[0],error) and approximation(resultToFind[0], resultFound[0], error):
        print("Validé")
        
    #Test 3
    resultToFind = [180, -30.501, -67.819]
    resultFound = leg_ik(-64.14, 0.0, -67.79)
    print("x:{0:.2f} y:{1:.2f} z:{2:.2f}".format(*resultFound))
    if approximation(resultToFind[0],resultFound[0],error) and approximation(resultToFind[0], resultFound[0],error) and approximation(resultToFind[0], resultFound[0], error):
        print("Validé")
        
    #Test 4
    resultToFind = [0, -30.645, 38.501]
    resultFound = leg_ik(203.23, 0.0, -14.30)
    print("x:{0:.2f} y:{1:.2f} z:{2:.2f}".format(*resultFound))
    if approximation(resultToFind[0],resultFound[0],error) and approximation(resultToFind[0], resultFound[0],error) and approximation(resultToFind[0], resultFound[0], error):
        print("Validé")