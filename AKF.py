'''
Author : woosang
Date : 16.9.2022
Name : kf_Positioning
Language : Python 3.9.12
License: GPLv2 License
'''

import numpy as np
from numpy.linalg import inv
from numpy.core.fromnumeric import transpose

from math import *

class AKF():
    def __init__(self):
        '''USER INPUT VARIABLES'''
        self.fst1, self.fst2, self.fst3, self.fst4 = True, True, True, True

        ''' Kalman Filter Variables '''
        ##Time Interval
        self.dt = 0.4
        ##State Vectors 
        self.X = np.array([[0.01],[0.01],[0.1],[0.1],[1],[1]])
        self.X1, self.X2, self.X3, self.X4 = self.X,self.X,self.X,self.X
        ##State Trasition Matrixs
        self.A = np.eye(6)+np.diag([self.dt for i in range(0,4)],k=2)+np.diag([pow(self.dt,2)/2 for i in range(0,2)],k=4)
        ##Error Covariance Matrix
        pa = np.eye(2)*(pow(self.dt,4)/4)
        pb = np.eye(2)*(pow(self.dt,3)/2)
        pc = np.eye(2)*(pow(self.dt,2)/2)
        pd = np.eye(2)*self.dt
        p1= np.hstack((pa,pb,pc))
        p2= np.hstack((pb,pc,pd))
        p3= np.hstack((pc,pd,np.eye(2)))
        P = np.vstack((p1,p2,p3)) # Init State Covariance
        self.P1, self.P2, self.P3, self.P4 = P,P,P,P
       
        #Process Noise Covariance 
        self.b = 0.
        self.Q = np.diag([0.01,0.01,0.25,0.16,0.25,0.16]) 
        # self.Q1, self.Q2, self.Q3, self.Q4 = Q, Q, Q, Q
        
        #Measurement Noise Covariance
        mul = 5
        self.r1, self.r2 = 0.1692 * mul, 0.1991 * mul
        r1_max, r2_max = self.r1 * 2, self.r2 * 2
        self.r1_gap, self.r2_gap = r1_max - self.r1, r2_max - self.r2 
        
        R = np.diag([self.r1 , self.r2]) 
        self.R1, self.R2, self.R3, self.R4 = R, R, R, R
        #Measurement Matrix
        self.H = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0]])
        
    def kf_update1(self, Measurement):
        '''1. Time Update("Predict")'''
        #1.1 Project the state ahead
        if (self.fst1 == True):
            self.pXk1 = self.A @ np.array([Measurement[0],Measurement[1],[0.1],[0.1],[1],[1]])
        else : self.pXk1 = self.A @ self.X1
            
        #1.2 Project the error covariance ahead
        self.pPk1 = (self.A @ self.P1 @ np.transpose(self.A)) + self.Q
   
        '''2. Measurement Update("Correct")'''
        #2.1 Compute the Kalman gain Kk
        self.Ksk1 = self.H @ self.pPk1 @ np.transpose(self.H) + self.R1
        self.Kk1 = self.pPk1 @ np.transpose(self.H) @ inv(self.Ksk1)
        #sub. Calc Sk(Yk - Y(k-1))
        self.Yk1 = self.H @ self.pXk1
        self.Sk1 = Measurement - self.Yk1

        #2.2 Update estimate with measurement
        self.X1 = self.pXk1 + np.dot(self.Kk1,self.Sk1)
        #2.3 Update the error covariance
        self.P1 = self.pPk1 - (self.Kk1 @ self.H @ self.pPk1)
        
        #2.4 Sage_Husa Adaptive Kalman Filter -> Q update
        # if self.cnt1 > 100 : self.Q1 = (1 - 1/self.cnt1) * self.Q1 + (1/self.cnt1) * (self.Kk1 @ self.Sk1 @ np.transpose(self.Sk1) @ np.transpose(self.Kk1) + self.P1 - self.pPk1 + self.Q1)
        # print("Updated Q1 : ", self.Q1)    
        
    def kf_update2(self, Measurement):
        if (self.fst2 == True):
            self.pXk2 = self.A @ np.array([Measurement[0],Measurement[1],[0.1],[0.1],[1],[1]])
        else : self.pXk2 = self.A @ self.X2

        self.pPk2 = (self.A @ self.P2 @ np.transpose(self.A)) + self.Q
        self.Ksk2 = self.H @ self.pPk2 @ np.transpose(self.H) + self.R2
        self.Kk2 = self.pPk2 @ np.transpose(self.H) @ inv(self.Ksk2)
        self.Yk2 = self.H @ self.pXk2
        self.Sk2 = Measurement - self.Yk2
        self.X2 = self.pXk2 + np.dot(self.Kk2,self.Sk2)
        self.P2 = self.pPk2 - (self.Kk2 @ self.H @ self.pPk2)
        # self.Q2 = (1 - 1/self.cnt2) * self.Q2 + (1/self.cnt2) * (self.Kk2 @ self.Sk2 @ np.transpose(self.Sk2) @ np.transpose(self.Kk2) + self.P2 - self.pPk2 + self.Q2)
        
    def kf_update3(self, Measurement):
        if (self.fst3 == True):
            self.pXk3 = self.A @ np.array([Measurement[0],Measurement[1],[0.1],[0.1],[1],[1]])
        else : self.pXk3 = self.A @ self.X3
        
        self.pPk3 = (self.A @ self.P3 @ np.transpose(self.A)) + self.Q
        self.Ksk3 = self.H @ self.pPk3 @ np.transpose(self.H) + self.R3
        self.Kk3 = self.pPk3 @ np.transpose(self.H) @ inv(self.Ksk3)
        self.Yk3 = self.H @ self.pXk3
        self.Sk3 = Measurement - self.Yk3
        self.X3 = self.pXk3 + np.dot(self.Kk3,self.Sk3)
        self.P3 = self.pPk3 - (self.Kk3 @ self.H @ self.pPk3)
        # self.Q3 = (1 - 1/self.cnt3) * self.Q2 + (1/self.cnt3) * (self.Kk3 @ self.Sk3 @ np.transpose(self.Sk3) @ np.transpose(self.Kk3) + self.P3 - self.pPk3 + self.Q3)
        
    def kf_update4(self, Measurement):
        if (self.fst4 == True):
            self.pXk4 = self.A @ np.array([Measurement[0],Measurement[1],[0.1],[0.1],[1],[1]])
        else : self.pXk4 = self.A @ self.X4

        self.pPk4 = (self.A @ self.P4 @ np.transpose(self.A)) + self.Q
        self.Ksk4 = self.H @ self.pPk4 @ np.transpose(self.H) + self.R4
        self.Kk4 = self.pPk4 @ np.transpose(self.H) @ inv(self.Ksk4)
        self.Yk4 = self.H @ self.pXk4
        self.Sk4 = Measurement - self.Yk4
        self.X4 = self.pXk4 + np.dot(self.Kk4,self.Sk4)
        self.P4 = self.pPk4 - (self.Kk4 @ self.H @ self.pPk4)
        # self.Q4 = (1 - 1/self.cnt4) * self.Q4 + (1/self.cnt4) * (self.Kk4 @ self.Sk4 @ np.transpose(self.Sk4) @ np.transpose(self.Kk4) + self.P4 - self.pPk4 + self.Q4)
        