import numpy as np
from math import pi

def angleMap(angle):
    while angle > pi:
        angle -= 2*pi
    while angle <= -pi:
        angle += 2*pi
    return angle

def qCalcError(array1, array2):
    sum = 0
    for i in range(6):
        sum += np.abs(angleMap(array1[i] - array2[i]))
    sum = sum / 6
    return sum

def trajPlanning(start,end,start_vel,end_vel,start_acc,end_acc,t,time) :
    if t < time:
        kMatrix = np.matrix(np.empty(shape=[len(start),6]))
        for i in range(len(start)) :
            theta0 = start[i]
            theta1 = end[i]
            v0 = start_vel[i]
            v1 = end_vel[i]
            a0 = start_acc[i]
            a1 = end_acc[i]

            kMatrix[i,0] = (12 * theta1 - 12 * theta0 - 6*(v0+v1)*time - (a0-a1) * time**2) / (2*time**5)
            kMatrix[i,1] = (30 * theta0 - 30 * theta1 + (14*v1 + 16*v0)*time + (3*a0- 2*a1) * time**2) / (2*time**4)
            kMatrix[i,2] = (20 * theta1 - 20 * theta0 - (8*v1 + 12*v0)*time - (3*a0- a1) * time**2) / (2*time**3)
            kMatrix[i,3] = a0 / 2
            kMatrix[i,4] = v0
            kMatrix[i,5] = theta0
        
        timeVector = np.matrix([t**5, t**4, t**3, t**2, t, 1]).T
        x = (kMatrix * timeVector).T.A[0]
    else:
        x = end
    
    return x