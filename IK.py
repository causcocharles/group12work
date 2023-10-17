import numpy as np
import math

global q_limit
q_limit = np.array([[-200.0,200.0],[-90.0,90.0],[-120.0,120.0],[-150.0,150.0],[-150.0,150.0],[-180.0,180.0]])
q_limit *= np.pi/180.0
def forward_kinematics(qlist):
    # DH参数
    d1 = 0.230
    a2 = 0.185
    a3 = 0.170
    d4 = 0.023
    d5 = 0.077
    d6 = 0.0855
    
    # 各关节相对于上一个关节的转动角度
    theta = np.array([qlist[0], qlist[1], qlist[2], qlist[3], qlist[4], qlist[5]])
    theta[1] -= np.pi/2
    theta[3] += np.pi/2
    theta[4] += np.pi/2
    # 标准DH参数表
    dh_params = np.array([
        [0, -np.pi/2, d1, theta[0]],
        [a2, 0, 0, theta[1]],
        [a3, 0, 0, theta[2]],
        [0, np.pi/2, d4, theta[3]],
        [0, np.pi/2, d5, theta[4]],
        [0, 0, d6, theta[5]]
    ])
    
    # 计算转换矩阵
    T = np.eye(4)
    for i in range(6):
        a, alpha, d, theta_i = dh_params[i]
        T_i = np.array([
            [np.cos(theta_i), -np.sin(theta_i)*np.cos(alpha), np.sin(theta_i)*np.sin(alpha), a*np.cos(theta_i)],
            [np.sin(theta_i), np.cos(theta_i)*np.cos(alpha), -np.cos(theta_i)*np.sin(alpha), a*np.sin(theta_i)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        T = np.dot(T, T_i)
    
    # 从转换矩阵提取位置和姿态
    position = T[:3, 3]
    rotation = T[:3, :3]

    return position, rotation
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def Pose2T(pose) :
    px = pose[0]
    py = pose[1]
    pz = pose[2]
    alpha = pose[3]
    beta = pose[4]
    gamma = pose[5]
    
    sx = math.sin(alpha)
    cx = math.cos(alpha)
    sy = math.sin(beta)
    cy = math.cos(beta)
    sz = math.sin(gamma)
    cz = math.cos(gamma)

    T = np.array([[cy*cz, -cy*sz, sy, px], 
                  [sx*sy*cz + cx*sz, -sx*sy*sz + cx*cz, -sx*cy, py],
                  [-cx*sy*cz + sx*sz, cx*sy*sz + sx*cz, cx*cy, pz],
                  [0, 0, 0, 1]
                ])
    return T
#限制绕Y'轴旋转的角度区间为 -90°~90° 如果刚好为-90°或90°，则
def Rot2Euler(R) :

    assert(isRotationMatrix(R))

    c_y = math.sqrt(R[0,0] * R[0,0] +  R[0,1] * R[0,1])

    singular = c_y < 1e-6

    if not singular :
        x = math.atan2(-R[1,2]/c_y,R[2,2]/c_y)
        y = math.atan2(R[0,2],c_y)
        z = math.atan2(-R[0,1]/c_y,R[0,0]/c_y)
    else:
        if R[0,2] > 0 :
            x = 0
            y = np.pi / 2
            z = math.atan2(R[1,0],R[1,1])
        else :
            x = 0
            y = -np.pi / 2
            z = -math.atan2(R[2,1],R[2,0])
    return np.array([x, y, z])

def inverse_kinematics(T) :
    pose = np.array(T)
    np.reshape(pose,(4,4))

    qlist = []
    d1 = 0.230
    a2 = 0.185
    a3 = 0.170
    d4 = 0.023
    d5 = 0.077
    d6 = 0.0855

    nx,ny,nz = pose[0,0],pose[1,0],pose[2,0]
    ox,oy,oz = pose[0,1],pose[1,1],pose[2,1]
    ax,ay,az = pose[0,2],pose[1,2],pose[2,2]
    px,py,pz = pose[0,3],pose[1,3],pose[2,3]

    #theta1
    theta1 = []
    m = -d6*ay + py
    n = px - ax*d6
    sum = m*m+n*n-d4*d4
    if sum < 0 :
        print("m*m+n*n-d4*d4 < 0 : can't solve")
        return 
    theta1.append(math.atan2(m,n) - math.atan2(d4,math.sqrt(m*m+n*n-d4*d4)))
    theta1.append(math.atan2(m,n) - math.atan2(d4,-math.sqrt(m*m+n*n-d4*d4)))

    #theta156

    theta156 = []
    for i in range(2) :
        s1 = math.sin(theta1[i])
        c1 = math.cos(theta1[i])
        sum = ax*s1 - ay*c1
        if sum < -1 or sum > 1 :
            if i == 1 :
                print("No suitable solve")
                return
            continue
        m = ny*math.cos(theta1[i]) - nx*math.sin(theta1[i])  
        n = oy*math.cos(theta1[i]) - ox*math.sin(theta1[i])
        if m*m + n*n - d4*d4 == 0 :
            print("m*m + n*n -d4*d4 == 0 : can't solve singular pose")
        theta5 = math.acos(sum)
        s5 = math.sin(theta5)
        if s5 == 0 :
            print("s5 == 0 : can't solve singular pose")
            return
        theta6 = math.atan2(m,n) - math.atan2(s5,0)
        theta156.append([theta1[i],theta5,theta6])

        theta5 = -math.acos(sum)
        s5 = math.sin(theta5)
        theta6 = math.atan2(m,n) - math.atan2(s5,0)
        theta156.append([theta1[i],theta5,theta6])
    
    #theta3、theta2、theta4
    for i in range(len(theta156)) :
        q1 = theta156[i][0]
        q5 = theta156[i][1]
        q6 = theta156[i][2]
        s1 = math.sin(q1)
        c1 = math.cos(q1)
        s6 = math.sin(q6)
        c6 = math.cos(q6)
        m = -d5*(s6*(nx*c1 + ny*s1) + c6*(ox*c1 + oy*s1)) + px*c1 - d6*(ax*c1 + ay*s1) + py*s1
        n = -d5*(-nz*s6 - oz*c6) - pz + d1 + az*d6
        sum = (m*m + n*n - a2*a2 -a3*a3) / (2*a2*a3)
        if sum < -1 or sum > 1 :
            if i == len(theta156) - 1 :
                print("No suitable solve")
                return
            continue
        #theta3
        q3 = math.acos(sum)
        #theta2
        s3 = math.sin(q3)
        c3 = math.cos(q3)
        s2 = ((a3*c3 + a2)*n - a3*s3*m)/(a2*a2 + a3*a3 + 2*a2*a3*c3)
        c2 = (m + a3*s3*s2)/(a3*c3 + a2)
        q2 = math.atan2(s2,c2)
        #theta4
        q4 = math.atan2(s6*(nx*c1 + ny*s1) + c6*(ox*c1 + oy*s1), nz*s6 + oz*c6) - q2 - q3
        qlist.append([q1,q2+np.pi/2,q3,q4-np.pi/2,q5-np.pi/2,q6])
        # print([q1,q2,q3,q4,q5,q6])
        sum = (m*m + n*n - a2*a2 -a3*a3) / (2*a2*a3)
        q3 = -math.acos(sum)
        s3 = math.sin(q3)
        c3 = math.cos(q3)
        s2 = ((a3*c3 + a2)*n - a3*s3*m)/(a2*a2 + a3*a3 + 2*a2*a3*c3)
        c2 = (m + a3*s3*s2)/(a3*c3 + a2)
        q2 = math.atan2(s2,c2)
        q4 = math.atan2(s6*(nx*c1 + ny*s1) + c6*(ox*c1 + oy*s1), nz*s6 + oz*c6) - q2 - q3
        qlist.append([q1,q2+np.pi/2,q3,q4-np.pi/2,q5-np.pi/2,q6])
        # print([q1,q2,q3,q4,q5,q6])
    qlist_final = []
    for i in range(len(qlist)) :
        if limit(qlist[i]) :
            qlist_final.append(qlist[i])
    return np.array(qlist_final)
       
def limit(q) :
    for i in range(len(q)) :
        if i == 0 :
            if q[i] < q_limit[i][0] :
                q[i] += 2*np.pi
            elif q[i] > q_limit[i][1] :
                q[i] -= 2*np.pi
        else :
            if q[i] > np.pi :
                q[i] -= 2*np.pi
            elif q[i] < -np.pi :
                q[i] += 2*np.pi 
            if q[i] < q_limit[i][0] or q[i] > q_limit[i][1] :
                return False
    return True

q = [[np.pi/6, 0, np.pi/6, 0, np.pi/3, 0],
    [np.pi/6, np.pi/6, np.pi/3, 0, np.pi/3, np.pi/6],
    [np.pi/2, 0, np.pi/2, np.pi/3, np.pi/3, np.pi/6],
    [-np.pi/6, np.pi/6, -np.pi/3, 0, np.pi/12, np.pi/2],
    [np.pi/12, np.pi/12, np.pi/12, np.pi/12, np.pi/12, np.pi/12]]

pose = [[0.117,0.334,0.499,-2.019,-0.058,-2.190],
        [-0.066,0.339,0.444,-2.618,-0.524,-3.141],
        [0.3,0.25,0.26,-2.64,0.59,-2.35],
        [0.42,0,0.36,3.14,1,-1.57],
        [0.32,-0.25,0.16,3,0.265,-0.84]]

for i in range(5):
    position,R = forward_kinematics(q[i])
    rpy = Rot2Euler(R)
    print("第"+str(i)+"组末端位置 "+str(position)+"\n"+"第"+str(i)+"组姿态 "+ str(rpy*(180.0/np.pi)) +"\n")
for i in range(5):   
    print("第"+str(i)+"组位姿对应关节角")
    print(inverse_kinematics(Pose2T(pose[i])))
