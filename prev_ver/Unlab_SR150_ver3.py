import os, sys
import csv
import random
import signal
import socket
import threading
import time
import datetime
from fxpmath import Fxp
from Parse_DNN import *
from EKF_AoA import *

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
        ''' Set Baudrate '''
        self.scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6) 
        self.delay = 3 # delay setting(sec)

        '''USER INPUT VARIABLES'''
        self.ekf = EKF_AoA() # EKF Object Creation
        # self.ref = np.array([0.0001, 1.25]) # reference position of tag
        # h_anc = 1.20 # height of anchor(SECC)
        # h_tag = 1.15 # height of tag(CAR)
        # self.h_diff = h_anc - h_tag
        
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
        time.sleep(self.delay)
        
        ## Ranging start ##
        state_ntf_rx = serial_trx(self.scpi_rx, "UWB MTRESP ON\r\n") # Responder Session start Command
        print(state_ntf_rx)
        time.sleep(self.delay)

        while 1: 
            self.scpi_ret = serial_rx(self.scpi_rx)
            
            try:
                ## Data Parsing ##
                result = self.scpi_ret.split(' ')
                session_id = result[0]
                distance = Fxp(val="0x"+result[5]+"0x"+result[4], signed=False, n_word=16, n_frac=0).astype(int).tolist() - 10
                AoA_azimuth = Fxp(val="0x"+result[7]+"0x"+result[6], signed=True, n_word=16, n_frac=7).astype(float)
                PDoA_azimuth = Fxp(val="0x"+result[9]+"0x"+result[8], signed=True, n_word=16, n_frac=7).astype(float)
                nlos = Fxp(val="0x"+result[10], signed = False, n_word=8, n_frac = 0).astype(int).tolist()
                
                ## convert types for dist and angle 
                # dist = math.sqrt(math.pow(float(distance)/100,2) - math.pow(self.h_diff,2))
                dist = float(distance)/100
                angle = math.pi * (float(AoA_azimuth)+90)/180
                s_dist = str(dist)
                
                # ## calculate position of TAGs
                x = dist * math.cos(angle)
                y = dist * math.sin(angle)
                x_ref = str(x)
                y_ref = str(y)
                # r_X2Y2 = pow((x - self.ref[0]),2) + pow((y - self.ref[1]),2)
                # r_err = str(r_X2Y2)
                if  result[0] == '11':
                    meas = np.array([[x],[y]])
                    self.ekf.ekf_update1(meas)
                    self.ekf.cnt1 = 1
                    print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,Fore.GREEN,"TAG 1 EKF : ({:.2f}, {:.2f})".format(self.ekf.X1[0][0],self.ekf.X1[1][0]),"\n",Fore.RESET)
                    
                elif  result[0] == '22':
                    meas = np.array([[x],[y]])
                    self.ekf.ekf_update2(meas)
                    self.ekf.cnt2 = 1
                    print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,"TAG 2 EKF : ({:.2f}, {:.2f})".format(self.ekf.X2[0][0],self.ekf.X2[1][0]),"\n")
                    
                elif  result[0] == '33':
                    meas = np.array([[x],[y]])
                    self.ekf.ekf_update3(meas)
                    self.ekf.cnt3 = 1
                    print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,"TAG 3 EKF : ({:.2f}, {:.2f})".format(self.ekf.X3[0][0],self.ekf.X3[1][0]),"\n")
                    
                elif  result[0] == '44':
                    meas = np.array([[x],[y]])
                    self.ekf.ekf_update4(meas)
                    self.ekf.cnt4 = 1
                    print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,"TAG 4 EKF : ({:.2f}, {:.2f})".format(self.ekf.X4[0][0],self.ekf.X4[1][0]),"\n")
                    
                else : pass
                # x_pos = self.X[0,0]
                # y_pos = self.X[1,0]
                # e_X2Y2 = pow((x_pos - self.ref[0]),2) + pow((y_pos - self.ref[1]),2)
                # e_err = str(e_X2Y2)
                
                # self.plot()     
            except:
                pass
            # # print(Fore.GREEN, x_ref, y_ref, scpi_ret,Fore.RESET)
            
            # ## save data(.csv file) ##
            # save_csv(ranging_result_csvF, [session_id, s_dist, x_pos, y_pos, x_ref, y_ref,aoa_azimuth, pdoa_azimuth])
            # save_csv(ranging_result_csvF, [session_id, s_dist, x_pos, y_pos, x_ref, y_ref,aoa_azimuth, pdoa_azimuth, e_err, r_err])
            # time.sleep(self.delay)
            self.scpi_rx.flush()
            result.clear()


if __name__ == "__main__": 
    # now = datetime.datetime.now()
    # nowDatetime = now.strftime('%Y_%m_%d_%H_%M_%S')
    
    # ranging_result_csvF = 'results/UWB_SR150_ranging_test_result-%s.csv' %nowDatetime
    # save_csv(ranging_result_csvF, ['Session_ID','Distance','pos_X','pos_Y','ref_X','ref_Y','AoA_azimuth','PDoA_azimuth'])
    # save_csv(ranging_result_csvF, ['Session_ID','Distance','pos_X','pos_Y','ref_X','ref_Y','AoA_azimuth','PDoA_azimuth', 'Estimated_Err', 'Ref_Err'])
    
    unlab = Unlab_SR150_Resp()
    unlab.Positioning()