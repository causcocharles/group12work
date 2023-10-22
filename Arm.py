import IK.IK as IK
import kinematics as ki
from numpy import mat
import numpy as np
import math
import tools

class Arm:
    obj_num = 4
    height = 0.08
    line_divide_n = 50
## Time spent
    # 0:pullDown 1:suck 2:pullUp 3:toStart 4:toEnd 5:toPlatform 6:putDown 7:loose 8:init
    period_lst = [0]*9
    one_round_period = 0
    round_num = 0
    stage = 0
    key_frame_t_lst = []    # time of key frame in one round
## key frame pose and position
    PAP_keyframe_lst = []
## key frame joint angles
    q_keyframe_lst = []
## last pose and position and q
    last_q = np.array([0,0,0,0,0,0])
## current q and current state: output variable
    cur_q = np.array([0,0,0,0,0,0])
    cur_state = True
    
    def __init__(self, PAP_lst):
        self.PAP_keyframe_lst = PAP_lst
        self.period_lst[0] = 2
        self.period_lst[1] = 0.1
        self.period_lst[2] = 1
        self.period_lst[3] = 5
        self.period_lst[4] = 7
        self.period_lst[5] = 3
        self.period_lst[6] = 1
        self.period_lst[7] = 0.1
        self.period_lst[8] = 4
        self.one_round_period = sum(self.period_lst)
        self.round_num = 0
        for i in range(10):
            self.key_frame_t_lst.append(sum(self.period_lst[:i]))
            
        # 0
        q_start_lst = [self.getClosestQ(self.PAP_keyframe_lst[0], np.array([0,0,0,0,0,0])),\
                        self.getClosestQ(self.PAP_keyframe_lst[1], np.array([0,0,0,0,0,0])),\
                        self.getClosestQ(self.PAP_keyframe_lst[2], np.array([0,0,0,0,0,0])),\
                        self.getClosestQ(self.PAP_keyframe_lst[3], np.array([0,0,0,0,0,0]))]
        
        # 1
        q_start_up_lst = [self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[0], 0.08), q_start_lst[0]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[1], 0.08), q_start_lst[1]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[2], 0.08), q_start_lst[2]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[3], 0.08), q_start_lst[3])]
        
        # 2
        # matrix = ki.eulerToMatrix(self.PAP_keyframe_lst[4][0],self.PAP_keyframe_lst[4][1],self.PAP_keyframe_lst[4][2],\
        #                         self.PAP_keyframe_lst[4][3],self.PAP_keyframe_lst[4][4],self.PAP_keyframe_lst[4][5])
        # for i in range(3):
        #     for j in range(3):
        #         matrix[i,j] = -matrix[i,j]
        # self.PAP_keyframe_lst[4] = ki.matrixToEuler(matrix)
        q_pond_start_lst = [self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[4],0.05), q_start_up_lst[0]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[4],0), q_start_up_lst[1]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[4],0), q_start_up_lst[2]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[4],0.05), q_start_up_lst[3])]
        
        # 3
        q_pond_end_lst = [self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[5],0.05), q_pond_start_lst[0]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[5],0), q_pond_start_lst[1]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[5],0), q_pond_start_lst[2]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[5],0.05), q_pond_start_lst[3])]
        
        # 4
        q_end_up_lst = [self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[6],0.08), q_pond_end_lst[0]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[6],0.05), q_pond_end_lst[1]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[6],0.05), q_pond_end_lst[2]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[6],0.15), q_pond_end_lst[3])]
        
        # 5
        q_end_lst = [self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[6],0.05), q_end_up_lst[0]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[6],0.03), q_end_up_lst[1]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[6],0.03), q_end_up_lst[2]),\
                        self.getClosestQ(self.getUpPAP(self.PAP_keyframe_lst[6],0.075), q_end_up_lst[3])]
        
        # 6
        q_init_lst = [np.array([0,0,0,0,0,0])]*4
        
        self.q_keyframe_lst = [q_start_lst, q_start_up_lst, q_pond_start_lst, q_pond_end_lst, q_end_up_lst, q_end_lst, q_init_lst]
        
    
    def getUpPAP(self, PAP, height):
        up_PAP = PAP.copy()
        up_PAP[2] += height
        return up_PAP
    
    # find the stage of the moment in one round
    def findStage(self, t):
        t_in_round = t - math.floor(t/self.one_round_period)*self.one_round_period
        for i in range(10):
            if t_in_round < self.key_frame_t_lst[i]:
                break
        return i-1
    
    def getClosestQ(self, PAP, last_q):
        q_lst = ki.coordsPoseToJoint(ki.eulerToMatrix(PAP[0],PAP[1],PAP[2],PAP[3],PAP[4],PAP[5]))
        sum_lst=[]
        for i in range(len(q_lst)):
            sum_lst.append(tools.qCalcError(q_lst[i], last_q))
            print(i)
            
        return q_lst[sum_lst.index(min(sum_lst))]
    
    def calc_q_and_state(self, t):
        self.last_q = self.cur_q
        
        # calculate round number and stage number
        self.round_num = math.floor(t/self.one_round_period)
        stage = self.findStage(t)
        if self.round_num > 3:
            return
        # print(f'{self.round_num}.{stage}')
        
        # determine sucker state according to stage
        if stage != 7 and stage != 8: 
            self.cur_state = True
        else: 
            self.cur_state = False
            
        # determine sucker state according to stage
        if stage == 1 or stage == 7:
            pass
        
        # pass the pond
        elif stage == 4:
            # point_n = self.PAP_keyframe_lst[4].copy()
            # point_n[0] = self.PAP_keyframe_lst[4][0] + t/self.period_lst[4]*(self.PAP_keyframe_lst[5][0]-self.PAP_keyframe_lst[4][0])
            # self.cur_q = self.getClosestQ(point_n, self.last_q)
            q1 = self.q_keyframe_lst[2][self.round_num]
            q2 = self.q_keyframe_lst[3][self.round_num]
            delta_t = self.period_lst[stage]
            stage_t = t - self.round_num * self.one_round_period - self.key_frame_t_lst[stage]  
            self.cur_q = tools.trajPlanning(q1, q2,[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0], stage_t, delta_t)
        # other cases: plan in joint space
        else:
            # pullDown
            if stage == 0:
                q1 = self.q_keyframe_lst[6][self.round_num]
                q2 = self.q_keyframe_lst[0][self.round_num]
                # delta_t = self.period_lst[0]
                # stage_t = t - self.round_num * self.one_round_period
                
            # pullUp
            elif stage == 2:
                q1 = self.q_keyframe_lst[0][self.round_num]
                q2 = self.q_keyframe_lst[1][self.round_num]
                
            # toStart
            elif stage == 3:
                q1 = self.q_keyframe_lst[1][self.round_num]
                q2 = self.q_keyframe_lst[2][self.round_num]
                
            # toEnd
            elif stage == 4:
                q1 = self.q_keyframe_lst[2][self.round_num]
                q2 = self.q_keyframe_lst[3][self.round_num]
                
            # toPlatform
            elif stage == 5:
                q1 = self.q_keyframe_lst[3][self.round_num]
                q2 = self.q_keyframe_lst[4][self.round_num]
               
            # putDown
            elif stage == 6:
                q1 = self.q_keyframe_lst[4][self.round_num]
                q2 = self.q_keyframe_lst[5][self.round_num] 
                
            # init
            elif stage == 8:
                q1 = self.q_keyframe_lst[5][self.round_num]
                q2 = self.q_keyframe_lst[6][self.round_num] 
                
            delta_t = self.period_lst[stage]
            stage_t = t - self.round_num * self.one_round_period - self.key_frame_t_lst[stage]    
            self.cur_q = tools.trajPlanning(q1, q2,[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0], stage_t, delta_t)
            
        
        