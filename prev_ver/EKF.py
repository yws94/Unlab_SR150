'''
Author : woosang
Date : 16.9.2022
Name : RTLS_EKF_ToA/AoA
Language : Python 3.9.12
License: GPLv2 License
'''

import numpy as np
from numpy.linalg import inv
from numpy.core.fromnumeric import transpose

from math import *

class EKF():
    def __init__(self):
        self.cnt = 0
        
        '''USER INPUT VARIABLES'''
        # self.ref = np.array([0.0001, 1.25]) # reference position of tag
        # h_anc = 1.20 # height of anchor(SECC)
        # h_tag = 1.15 # height of tag(CAR)
        # self.h_diff = h_anc - h_tag

        ''' Kalman Filter Variables '''
        ##Time Interval
        self.dt = 0.4
        ##State Vectors 
        self.X = np.array([[0.01],[0.01],[0.1],[0.1],[1],[1]])
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
        self.P = np.vstack((p1,p2,p3)) # Init State Covariance
       
        #Process Noise Covariance 
        self.Q = np.diag([0.01,0.01,0.25,0.16,0.25,0.16]) 

        #Measurement Noise Covariance
        r1 = 0.1692
        r2 = 0.1991
        mul = 5
        self.R = np.diag([r1 * mul, r2 * mul]) 
        
        #Measurement Matrix
        self.H = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0]])
        
    def ekf_update(self, Measurement):
        '''1. Time Update("Predict")'''
        #1.1 Project the state ahead
        if (self.cnt == 0):
            X = np.array([[Measurement[0,0]],[Measurement[1,0]],[0.1],[0.1],[1],[1]])
            self.pXk = self.A @ X
        else : self.pXk = self.A @ self.X
            
        #1.2 Project the error covariance ahead
        self.pPk = (self.A @ self.P @ np.transpose(self.A)) + self.Q 
            
        '''2. Measurement Update("Correct")'''
        #2.1 Compute the Kalman gain Kk
        self.Ksk = self.H @ self.pPk @ np.transpose(self.H) + self.R
        self.Kk = self.pPk @ np.transpose(self.H) @ inv(self.Ksk)
        #sub. Calc Sk(Yk - Y(k-1))
        self.Yk = self.H @ self.pXk
        self.Sk = Measurement - self.Yk

        #2.2 Update estimate with measurement
        self.X = self.pXk + np.dot(self.Kk,self.Sk)
        #2.3 Update the error covariance
        self.P = self.pPk - (self.Kk @ self.H @ self.pPk)