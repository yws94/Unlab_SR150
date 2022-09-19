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

##Time Interval
dt = 0.4
##State Trasition Matrixs
A = np.eye(6)+np.diag([dt for i in range(0,4)],k=2)+np.diag([pow(dt,2)/2 for i in range(0,2)],k=4)

#Process Noise Covariance 
Q = np.diag([0.01,0.01,0.25,0.16,0.25,0.16]) 

#Measurement Noise Covariance
r1 = 0.1692
r2 = 0.1991
mul = 5
R = np.diag([r1 * mul, r2 * mul]) 

#Measurement Matrix
H = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0]])
    
def ekf_update(Measurement, X, P, cnt):
    '''1. Time Update("Predict")'''
    #1.1 Project the state ahead
    if (cnt == 0):
        X = np.array([[Measurement[0,0]],[Measurement[1,0]],[0.1],[0.1],[1],[1]])
        pXk = A @ X
    else : pXk = A @ X
        
    #1.2 Project the error covariance ahead
    pPk = (A @ P @ np.transpose(A)) + Q 
        
    '''2. Measurement Update("Correct")'''
    #2.1 Compute the Kalman gain Kk
    Ksk = H @ pPk @ np.transpose(H) + R
    Kk = pPk @ np.transpose(H) @ inv(Ksk)
    #sub. Calc Sk(Yk - Y(k-1))
    Yk = H @ pXk
    Sk = Measurement - Yk

    #2.2 Update estimate with measurement
    Xk = pXk + np.dot(Kk,Sk)
    #2.3 Update the error covariance
    Pk = pPk - (Kk @ H @ pPk)
    
    return Xk, Pk