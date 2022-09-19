import os, sys
import csv
import random
import signal
import socket
import threading
import time
import datetime
from fxpmath import Fxp

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
# --------------------------------------NO EDITS BELOW------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

def serial_tx(my_port, command):
    b_command = command.encode()
    my_port.write(b_command)

def serial_rx(my_port):
    line = my_port.read_until()
    line = line.strip()
    return line.decode("utf-8")

def serial_trx(my_port, command):
    serial_tx(my_port, command)
    return (serial_rx(my_port))

def save_csv(csvf, row):
    with open(csvf, "a", newline="") as F:
        w = csv.writer(F)
        w.writerow(row)
        
class Unlab_SR150_Resp():
    def __init__(self):
        self.reset_delay = 3 # delay setting(sec)
        self.ranging_stop_delay = 1 # delay setting(Sec)
        self.delay = 1 # delay setting(sec)
        self.cnt = 0
        
        ## set baudrate ##
        self.scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6) 
        
        '''USER INPUT VARIABLES'''
        # self.ref = np.array([0.0001, 1.25]) # reference position of tag
        h_anc = 1.20 # height of anchor(SECC)
        h_tag = 1.15 # height of tag(CAR)
        self.h_diff = h_anc - h_tag

        ''' Kalman Filter Variables '''
        ##Time Interval
        self.dt = 1
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
        
    def plot(self):
        plt.cla()
        plt.xlabel('X(m)', labelpad=20, size=14)
        plt.ylabel('Y(m)', labelpad=20, size=14)
        plt.axis([-5, 5, 0, 5])
        plt.xticks(np.arange(-5, 5, 0.5))
        plt.yticks(np.arange(0, 5, 0.5))
        for i in range(0,10):
            plt.axhline((i+1)/2, -5, 5, color='lightgray', linestyle='--', linewidth=0.7)
        for i in range(0,20):
            plt.vlines((i-10)/2, 0, 5, color='lightgray', linestyle='--', linewidth=0.7)
        x = self.X[0,0]
        y = self.X[1,0]
        plt.scatter(x,y,color='r',s=450)
        plt.pause(0.1)
        
    def Positioning(self):
        ## Reset all ##
        state_ntf_rx = serial_trx(self.scpi_rx, "RST\r\n")
        print(state_ntf_rx)
        time.sleep(self.reset_delay)

        # state_ntf_rx = serial_trx(self.scpi_rx, "UWB DISOFFSET 30\r\n") # Input offset value
        # print(state_ntf_rx)
        # state_ntf_rx = serial_trx(self.scpi_rx, "UWB ANTPAIR 1\r\n") # Set PDOA offset to responder
        # print(state_ntf_rx)
        # state_ntf_rx = serial_trx(self.scpi_rx, "UWB PDOAOFFSET -50\r\n") # Input offset value
        # print(state_ntf_rx)

        ## Session #1 Ranging start ##
        state_ntf_rx = serial_trx(self.scpi_rx, "UWB MTRESP ON\r\n") # Responder of Session #1 start Command
        print(state_ntf_rx)
        time.sleep(self.delay)

        self.scpi_rx.close()
        # self.scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)
        # self.scpi_ret = serial_rx(self.scpi_rx)

        while 1: 
            self.scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)
            self.scpi_ret = serial_rx(self.scpi_rx)
            try:
                ## Data Parsing ##
                result = self.scpi_ret.split(' ')
                session_id = result[0]
                # distance = int(result[4],16)
                distance = Fxp(val="0x"+result[5]+"0x"+result[4], signed=False, n_word=16, n_frac=0).astype(int).tolist()
                AoA_azimuth = Fxp(val="0x"+result[7]+"0x"+result[6], signed=True, n_word=16, n_frac=7).astype(float)
                PDoA_azimuth = Fxp(val="0x"+result[9]+"0x"+result[8], signed=True, n_word=16, n_frac=7).astype(float)
                
                # distance2 = int(result[5],16)
            #     aoa_azimuth = result[2]
            #     aoa_elevation = result[3]
            #     pdoa_azimuth = result[4]
            #     pdoa_elevation = result[5].replace("\r\n", '')
            except:
                pass
            
            ## convert types for dist and angle 
            # dist = math.sqrt(math.pow(float(distance)/100,2) - math.pow(self.h_diff,2))
            # angle = math.pi * (float(aoa_azimuth)+90)/180
            # s_dist = str(dist)
            
            # ## calculate position of TAGs
            # x = dist * math.cos(angle)
            # y = dist * math.sin(angle)
            # x_ref = str(x)
            # y_ref = str(y)
            # # r_X2Y2 = pow((x - self.ref[0]),2) + pow((y - self.ref[1]),2)
            # # r_err = str(r_X2Y2)
            
            # meas = np.array([[x],[y]])
            # self.ekf_update(meas)
            # self.cnt = 1

            # x_pos = self.X[0,0]
            # y_pos = self.X[1,0]
            # e_X2Y2 = pow((x_pos - self.ref[0]),2) + pow((y_pos - self.ref[1]),2)
            # e_err = str(e_X2Y2)
            
            # self.plot()
            
            print(Fore.GREEN, session_id, distance, Fore.RESET)
            # # print(Fore.GREEN, x_ref, y_ref, scpi_ret,Fore.RESET)
            
            # ## save data(.csv file) ##
            # save_csv(ranging_result_csvF, [session_id, s_dist, x_pos, y_pos, x_ref, y_ref,aoa_azimuth, pdoa_azimuth])
            # save_csv(ranging_result_csvF, [session_id, s_dist, x_pos, y_pos, x_ref, y_ref,aoa_azimuth, pdoa_azimuth, e_err, r_err])
            # time.sleep(self.delay)
            self.scpi_rx.close()


if __name__ == "__main__": 
    # now = datetime.datetime.now()
    # nowDatetime = now.strftime('%Y_%m_%d_%H_%M_%S')
    
    # ranging_result_csvF = 'results/UWB_SR150_ranging_test_result-%s.csv' %nowDatetime
    # save_csv(ranging_result_csvF, ['Session_ID','Distance','pos_X','pos_Y','ref_X','ref_Y','AoA_azimuth','PDoA_azimuth'])
    # save_csv(ranging_result_csvF, ['Session_ID','Distance','pos_X','pos_Y','ref_X','ref_Y','AoA_azimuth','PDoA_azimuth', 'Estimated_Err', 'Ref_Err'])
    
    ekf = Unlab_SR150_Resp()
    ekf.Positioning()
    