import numpy, math
from numpy import mat
from math import pi, sin, cos, atan2, sqrt, acos

# M-DH参数
alpha1 = -pi/2
alpha4 = pi/2
alpha5 = pi/2
a2 = 0.185
a3 = 0.170
d1 = 0.23
d2 = 0.023
d5 = 0.077
d6 = 0.0855    
    
jointMax = [200/180*pi, 90/180*pi, 120/180*pi, 150/180*pi, 1180/180*pi, 180/180*pi]

def angleRestrict(solution):
    abandon_lst = []
    for i in range(len(solution)):
        for j in range(6):
            if solution[i][j] > jointMax[j] or solution[i][j] < -jointMax[j]:
                abandon_lst.append(solution[i])   
                break
    for sol in abandon_lst:
        solution.remove(sol)
    return
    
def sgn(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0
    
def angleMap(sol):
    for i in range(len(sol)):
        while sol[i] <= -pi:
            sol[i] += 2*pi
        while sol[i] > pi:
            sol[i] -= 2*pi
        if(numpy.abs(sol[i]) < 1.00e-5):
            sol[i] = 0
    return

def caliMat(matrix):
    rows, cols = matrix.shape
    for i in range(rows):
        for j in range(cols):
            if numpy.abs(matrix[i,j])<1.00e-10:
                matrix[i,j] = 0.000
    return

def angleToMatrix(alpha, a, theta, d): # x'y'z'欧拉角
    T = mat([[cos(theta), -sin(theta), 0, a],\
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha)],\
            [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha)],\
                [0,0,0,1]])
    caliMat(T)
    return T

def eulerToMatrix(x, y, z, rx, ry, rz):  # x'y'z'欧拉角
    matrix = mat([[cos(rz)*cos(ry), -cos(ry)*sin(rz), sin(ry), x],\
        [cos(rx)*sin(rz)+cos(rz)*sin(ry)*sin(rx), cos(rz)*cos(rx)-sin(rz)*sin(ry)*sin(rx), -cos(ry)*sin(rx), y],\
            [sin(rx)*sin(rz)-cos(rz)*cos(rx)*sin(ry), cos(rz)*sin(rx)+cos(rx)*sin(rz)*sin(ry), cos(ry)*cos(rx), z],\
                [0, 0, 0, 1]]) 
    return matrix

def matrixToEuler(matrix): 
    euler_list = [matrix[0,3], matrix[1,3], matrix[2,3], \
        atan2(-matrix[1,2], matrix[2,2])/pi*180,\
            atan2(matrix[0,2], math.sqrt(matrix[1,2]**2 + matrix[2,2]**2))/pi*180,\
                atan2(-matrix[0,1], matrix[0,0])/pi*180]
    return euler_list

def jointToCoordsPose(theta):
    T10 = angleToMatrix(0,0,theta[0],d1)
    T21 = angleToMatrix(alpha1,0,theta[1]-pi/2,d2)
    T32 = angleToMatrix(0,a2,theta[2],0)
    T43 = angleToMatrix(0,a3,theta[3]+pi/2,0)
    T54 = angleToMatrix(alpha4,0,theta[4]+pi/2,d5)
    T65 = angleToMatrix(alpha5,0,theta[5],d6)
    return T10*T21*T32*T43*T54*T65

def coordsPoseToJoint(T60):
    solution = []
    theta = [0,0,0,0,0,0]
    
    #坐标变换，将末端坐标变换到末端前，将原点坐标变换到第一个关节处
    T60[0,3] = T60[0,3] - T60[0,2] * d6
    T60[1,3] = T60[1,3] - T60[1,2] * d6
    T60[2,3] = T60[2,3] - T60[2,2] * d6 - d1
    
    # 求theta1，有2解
    theta[0] = acos(d2/sqrt(T60[0,3]**2 + T60[1,3]**2)) - atan2(T60[0,3], T60[1,3])
    solution.append(theta.copy())
    theta[0] = -acos(d2/sqrt(T60[0,3]**2 + T60[1,3]**2)) - atan2(T60[0,3], T60[1,3])
    solution.append(theta.copy())
    # 角度映射到 [0, 2pi)
    for sols in solution:
        angleMap(sols)
    
    #利用姿态求theta234/5/6
    new_solution = []
    for i in range(len(solution)):
        theta = solution[i]
        r13 = T60[0,2]*cos(theta[0]) + T60[1,2]*sin(theta[0])
        r21 = T60[1,0]*cos(theta[0]) - T60[0,0]*sin(theta[0])
        r22 = T60[1,1]*cos(theta[0]) - T60[0,1]*sin(theta[0])
        r23 = T60[1,2]*cos(theta[0]) - T60[0,2]*sin(theta[0])
        r33 = T60[2,2]
        theta[4] = atan2(sqrt(r21**2 + r22**2), -r23)
        while theta[4] < 0:
            theta[4] += pi
        while theta[4] > pi:
            theta[4] -= pi    
        tmp = sgn(sin(theta[4]))    # 等于0的情况未讨论
        theta[3] = atan2(-r33*tmp, r13*tmp)
        theta[5] = atan2(-r22*tmp, r21*tmp)
        angleMap(theta)
        if sqrt(r21**2 + r22**2)>1.00e-10:
            theta = solution[i].copy()
            theta[4] = atan2(-sqrt(r21**2 + r22**2), -r23)
            while theta[4] > 0:
                theta[4] -= pi
            while theta[4] < -pi:
                theta[4] += pi    
            tmp = sgn(sin(theta[4]))    # 等于0的情况未讨论
            theta[3] = atan2(-r33*tmp, r13*tmp)
            theta[5] = atan2(-r22*tmp, r21*tmp)
            angleMap(theta)
            new_solution.append(theta.copy()) 
    solution += new_solution
    new_solution.clear()
    abandon_solution = []
    
    # 利用theta234求theta2/3/4
    for sols in solution:
        f1 = T60[0,3]*cos(sols[0]) + T60[1,3]*sin(sols[0]) - d5*cos(sols[5])*(T60[0,1]*cos(sols[0]) + T60[1,1]*sin(sols[0])) - d5*sin(sols[5])*(T60[0,0]*cos(sols[0]) + T60[1,0]*sin(sols[0]))
        f2 = -(T60[2,3] - d5*T60[2,1]*cos(sols[5]) - d5*T60[2,0]*sin(sols[5]))
        # 余弦定理，若cos(theta3)>1则机械臂够不到，舍去
        if numpy.abs(f1**2 + f2**2 - a2**2 - a3**2) / (2*a2*a3) > 1:
            abandon_solution.append(sols)
            continue
        
        new_sols = sols.copy()
        sols[2] = acos((f1**2 + f2**2 - a2**2 - a3**2) / (2*a2*a3))
        g1 = (a2*f1 + a3*f2*sin(sols[2]) + a3*f1*cos(sols[2]))/(a3**2*sin(sols[2])**2 + a3**2*cos(sols[2])**2 + a2**2)
        g2 = (a2*f2 - a3*f1*sin(sols[2]) + a3*f2*cos(sols[2]))/(a3**2*sin(sols[2])**2 + a3**2*cos(sols[2])**2 + a2**2)
        sols[1] = atan2(g2, g1)
        sols[3] = sols[3] - sols[2] - sols[1]
        new_sols[2] = -acos((f1**2 + f2**2 - a2**2 - a3**2) / (2*a2*a3))
        g1 = (a2*f1 + a3*f2*sin(new_sols[2]) + a3*f1*cos(new_sols[2]))/(a3**2*sin(new_sols[2])**2 + a3**2*cos(new_sols[2])**2 + a2**2)
        g2 = (a2*f2 - a3*f1*sin(new_sols[2]) + a3*f2*cos(new_sols[2]))/(a3**2*sin(new_sols[2])**2 + a3**2*cos(new_sols[2])**2 + a2**2)
        new_sols[1] = atan2(g2, g1)
        new_sols[3] = new_sols[3] - new_sols[2] - new_sols[1]
        new_solution.append(new_sols)
    for sols in abandon_solution:
        solution.remove(sols)
    solution = solution + new_solution
    
    for sols in solution:
        sols[1] += pi/2
        sols[3] -= pi/2
        sols[4] -= pi/2
        angleMap(sols)
        angleRestrict(solution)
    return solution