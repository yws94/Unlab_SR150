import os, sys
import csv
import random
import signal
import socket
import threading
import time
import datetime
from fxpmath import Fxp
from multiprocessing import Process, Queue
from Parse_DNN import *

import numpy.matlib
import numpy as np
from numpy.linalg import inv
from numpy.core.fromnumeric import transpose

import math
from math import *

import pyvisa as visa
from colorama import Fore

import serial
import serial.tools.list_ports
import serial.serialutil

import matplotlib.pyplot as plt

# ---------------------------------TEST RUN CONFIGS---------------------------------------------------------------------

Rx_DEVICE_COM_PORT = 'com16' #responder COM Port

# ----------------------------------------------------------------------------------------------------------------------
dt = 0.4
X = np.array([[0.01],[0.01],[0.1],[0.1],[1],[1]])

##State Trasition Matrixs
A = np.eye(6)+np.diag([dt for i in range(0,4)],k=2)+np.diag([pow(dt,2)/2 for i in range(0,2)],k=4)

##Error Covariance Matrix
pa = np.eye(2)*(pow(dt,4)/4)
pb = np.eye(2)*(pow(dt,3)/2)
pc = np.eye(2)*(pow(dt,2)/2)
pd = np.eye(2)*dt
p1= np.hstack((pa,pb,pc))
p2= np.hstack((pb,pc,pd))
p3= np.hstack((pc,pd,np.eye(2)))
P = np.vstack((p1,p2,p3)) # Init State Covariance

#Process Noise Covariance 
Q = np.diag([0.01,0.01,0.25,0.16,0.25,0.16]) 

#Measurement Noise Covariance
r1 = 0.1692
r2 = 0.1991
mul = 5
R = np.diag([r1 * mul, r2 * mul]) 

#Measurement Matrix
H = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0]])

reset_delay = 3 # delay setting(sec)
scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)
    
# ----------------------------------------------------------------------------------------------------------------------

def serial_tx(my_port, command):
    b_command = command.encode()
    my_port.write(b_command)

def serial_rx(my_port):
    line = my_port.readline()
    line = line.strip()
    return line.decode("utf-8")

def serial_trx(my_port, command):
    serial_tx(my_port, command)
    return (serial_rx(my_port))

def save_csv(csvf, row):
    with open(csvf, "a", newline="") as F:
        w = csv.writer(F)
        w.writerow(row)

def Positioning():
    ekf_data = []
    DNN = []
    ## Reset all ##
    state_ntf_rx = serial_trx(scpi_rx, "RST\r\n")
    print(state_ntf_rx)
    time.sleep(reset_delay)
    
    state_ntf_rx = serial_trx(scpi_rx, "UWB MTRESP ON\r\n") # Responder of Session #1 start Command
    print(state_ntf_rx)
    time.sleep(reset_delay)
    
    while True: 
        scpi_ret = serial_rx(scpi_rx)
        
        try:
            result = scpi_ret.split(' ')
            
            ## EKF Data Parsing ##
            session_id = result[0]
            distance = Fxp(val="0x"+result[5]+"0x"+result[4], signed=False, n_word=16, n_frac=0).astype(int).tolist() - 10
            AoA_azimuth = Fxp(val="0x"+result[7]+"0x"+result[6], signed=True, n_word=16, n_frac=7).astype(float)
            # PDoA_azimuth = Fxp(val="0x"+result[9]+"0x"+result[8], signed=True, n_word=16, n_frac=7).astype(float)
            nlos = Fxp(val="0x"+result[10], signed = False, n_word=8, n_frac = 0).astype(int).tolist()
            
            ekf_data = [session_id, distance, AoA_azimuth, nlos]

            ## DNN Data Parsing ##
            # DNN = Parsing_DNN(result)
            # print(DNN)
            
            ## convert types for dist and angle ##
            # dist = float(distance)/100
            # angle = math.pi * (float(AoA_azimuth)+90)/180
            # # s_dist = str(dist)
            
            # ## calculate position of TAGs ##
            # x = dist * math.cos(angle)
            # y = dist * math.sin(angle)
            # # x_ref = str(x)
            # # y_ref = str(y)
            # meas[0][0], meas[1][0] = x, y
            print(ekf_data[0],"\n")
            
            # if (len(ekf_data) == 4) and (ekf_data[0] == '11'):
            #     print("TAG 1 : ({:.2f}, {:.2f})".format(x,y),"\n")

            # elif (len(ekf_data) == 4) and (ekf_data[0] == '22'):
            #     print("TAG 2 : ({:.2f}, {:.2f})".format(x,y),"\n")
                
            # elif (len(ekf_data) == 4) and (ekf_data[0] == '33'):
            #     print("TAG 3 : ({:.2f}, {:.2f})".format(x,y),"\n")
        
            # elif (len(ekf_data) == 4) and (ekf_data[0] == '44'):
            #     print("TAG 4 : ({:.2f}, {:.2f})".format(x,y),"\n")

            # else : pass  
        
        except:
            pass
        
            scpi_rx.flush()
            result.clear()
            ekf_data.clear()
            DNN.clear()
            time.sleep(0.1)
        # ## save data(.csv file) ##
        # save_csv(ranging_result_csvF, [session_id, s_dist, x_pos, y_pos, x_ref, y_ref,aoa_azimuth, pdoa_azimuth])
        # save_csv(ranging_result_csvF, [session_id, s_dist, x_pos, y_pos, x_ref, y_ref,aoa_azimuth, pdoa_azimuth, e_err, r_err])

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

def EKFLoop():
    time.sleep(6.1)  
    dist = float(ekf_data[1])/100
    angle = math.pi * (float(ekf_data[2])+90)/180
    print("Distance : ", dist, "\n")
    ## calculate position of TAGs ##
    x = dist * math.cos(angle)
    y = dist * math.sin(angle)
    meas[0][0], meas[1][0] = x, y
    
    if ekf_data[0] == '11':
        if cnt1 == 0 :
            X1, P1 = ekf_update(meas, X, P, cnt1)
            cnt1 = 1
        else :
            X1, P1 = ekf_update(meas, X1, P1, cnt1)
            print("TAG 1 : ({:.2f}, {:.2f})".format(X1[0][0],X1[1][0]),"\n")
            
    elif ekf_data[0] == '22':
        if cnt2 == 0 :
            X2, P2 = ekf_update(meas, X, P, cnt2)
            cnt2 = 1
        else :
            X2, P2 = ekf_update(meas, X2, P2, cnt2)
            print("TAG 2 : ({:.2f}, {:.2f})".format(X2[0][0],X2[1][0]),"\n")
        
    elif ekf_data[0] == '33':
        if cnt3 == 0 :
            X3, P3 = ekf_update(meas, X, P, cnt3)
            cnt3 = 1
        else :
            X3, P3 = ekf_update(meas, X3, P3, cnt3)
            print("TAG 3 : ({:.2f}, {:.2f})".format(X3[0][0],X3[1][0]),"\n")
            
    elif ekf_data[0] == '44':
        if cnt4 == 0 :
            X4, P4 = ekf_update(meas, X, P, cnt4)
            cnt4 = 1
        else :
            X4, P4 = ekf_update(meas, X4, P4, cnt4)
            print("TAG 4 : ({:.2f}, {:.2f})".format(X4[0][0],X4[1][0]),"\n")
    else : pass
    time.sleep(0.1)

            
if __name__ == "__main__": 
    # now = datetime.datetime.now()
    # nowDatetime = now.strftime('%Y_%m_%d_%H_%M_%S')
    ekf_data = [0,0,0,0]
    meas = np.array([[0],[0]])
    X1, X2, X3, X4 = X,X,X,X
    P1, P2, P3, P4 = P,P,P,P
    # ranging_result_csvF = 'results/UWB_SR150_ranging_test_result-%s.csv' %nowDatetime
    # save_csv(ranging_result_csvF, ['Session_ID','Distance','pos_X','pos_Y','ref_X','ref_Y','AoA_azimuth','PDoA_azimuth'])
    # save_csv(ranging_result_csvF, ['Session_ID','Distance','pos_X','pos_Y','ref_X','ref_Y','AoA_azimuth','PDoA_azimuth', 'Estimated_Err', 'Ref_Err'])
    cnt1, cnt2, cnt3, cnt4 = 0,0,0,0
    # t1 = threading.Thread(target = EKFLoop, daemon= True)
    # t1.start()
    # t1.join()
    Positioning()
