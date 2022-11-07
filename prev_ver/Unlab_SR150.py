
'''
Author : woosang
Date : 16.9.2022
Name : RTLS_EKF_ToA/AoA_UWB
Language : Python 3.9.12
License: GPLv2 License
'''

import csv
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

# ---------------------------------USER COM PORT---------------------------------------------------------------------

Rx_DEVICE_COM_PORT = 'com16' # Check Port #.number when SR150 module is connected 

# ----------------------------------------------------------------------------------------------------------------------

# --------------------------------------SERIAL FUNCTIONS----------------------------------------------------------------

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
        
# ----------------------------------------------------------------------------------------------------------------------  

## Storing data in CSV file  ##
def save_csv(csvf, row):
    with open(csvf, "a", newline="") as F:
        w = csv.writer(F)
        w.writerow(row)
        
# ----------------------------------------------------------------------------------------------------------------------  


# ----------------------------------------------------------------------------------------------------------------------  

''' CLASS : Read SR150' serial data and Store pos/ekf_pos in .csv format '''  
class Unlab_SR150_Resp():
    def __init__(self):
        ''' SET Baudrate '''
        self.scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6) 

        '''USER INPUT VARIABLES'''
        self.ekf = EKF_AoA() # EKF Object Creation
        self.delay = 3 # Waiting time(sec) Responder's reset 
        
    def Positioning(self):
        ''' RESET ALL '''
        state_ntf_rx = serial_trx(self.scpi_rx, "RST\r\n") # Responder 'Reset' Command
        print(state_ntf_rx)
        time.sleep(self.delay)
        
        ''' UWB Module ON, Ranging start '''
        state_ntf_rx = serial_trx(self.scpi_rx, "UWB MTRESP ON\r\n") # Responder 'Session start' Command
        print(state_ntf_rx)
        time.sleep(self.delay)

        while 1: 
            self.scpi_ret = serial_rx(self.scpi_rx)
            
            try:
                ''' Data Parsing '''
                result = self.scpi_ret.split(' ')
                tag_id = result[0]
                distance = Fxp(val="0x"+result[5]+"0x"+result[4], signed=False, n_word=16, n_frac=0).astype(int).tolist() - 10
                AoA_azimuth = Fxp(val="0x"+result[7]+"0x"+result[6], signed=True, n_word=16, n_frac=7).astype(float)
                PDoA_azimuth = Fxp(val="0x"+result[9]+"0x"+result[8], signed=True, n_word=16, n_frac=7).astype(float)
                nlos = Fxp(val="0x"+result[10], signed = False, n_word=8, n_frac = 0).astype(int).tolist()
                
                ''' Convert types for dist and angle '''
                dist = float(distance)/100
                angle = math.pi * (float(AoA_azimuth)+90)/180
                
                ''' Calculate position of TAGs '''
                x = dist * math.cos(angle)
                y = dist * math.sin(angle)
                x_ref = str(x)
                y_ref = str(y)

                if  tag_id == '11':
                    meas = np.array([[x],[y]])
                    self.ekf.ekf_update1(meas) # EKF Calculation (TAG 1)
                    self.ekf.cnt1 = 1
                    x1, y1 = self.ekf.X1[0][0], self.ekf.X1[1][0]
                    save_csv(ranging_result_csvF, [datetime.datetime.now().time(), tag_id, x1, y1, x_ref, y_ref, dist, AoA_azimuth])
                    print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,Fore.GREEN,"TAG 1 EKF : ({:.2f}, {:.2f})".format(x1, y1),"\n",Fore.RESET)
                    
                elif  tag_id == '22':
                    meas = np.array([[x],[y]])
                    self.ekf.ekf_update2(meas) # EKF Calculation (TAG 2)
                    self.ekf.cnt2 = 1
                    x2, y2 = self.ekf.X2[0][0], self.ekf.X2[1][0]
                    save_csv(ranging_result_csvF, [datetime.datetime.now().time(), tag_id, x2, y2, x_ref, y_ref, dist, AoA_azimuth])
                    print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,"TAG 2 EKF : ({:.2f}, {:.2f})".format(x2,y2),"\n")
                    
                elif  tag_id == '33':
                    meas = np.array([[x],[y]])
                    self.ekf.ekf_update3(meas) # EKF Calculation (TAG 3)
                    self.ekf.cnt3 = 1
                    x3, y3 = self.ekf.X3[0][0], self.ekf.X3[1][0]
                    save_csv(ranging_result_csvF, [datetime.datetime.now().time(), tag_id, x3, y3, x_ref, y_ref, dist, AoA_azimuth])
                    print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,"TAG 3 EKF : ({:.2f}, {:.2f})".format(x3,y3),"\n")
                    
                elif  tag_id == '44':
                    meas = np.array([[x],[y]])
                    self.ekf.ekf_update4(meas) # EKF Calculation (TAG 4)
                    self.ekf.cnt4 = 1
                    x4, y4 = self.ekf.X4[0][0], self.ekf.X4[1][0]
                    save_csv(ranging_result_csvF, [datetime.datetime.now().time(), tag_id, x4, y4, x_ref, y_ref, dist, AoA_azimuth])
                    print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,"TAG 4 EKF : ({:.2f}, {:.2f})".format(x4,y4),"\n")
                else : pass
            except:
                pass

            self.scpi_rx.flush()
            result.clear()


if __name__ == "__main__": 
    now = datetime.datetime.now()
    nowDatetime = now.strftime('%Y_%m_%d_%H_%M_%S')
    
    ranging_result_csvF = 'UWB_SR150_ranging_test_result-%s.csv' %nowDatetime
    save_csv(ranging_result_csvF, ['Time','TAG_ID','EKF_X','EKF_Y','Ref_X','Ref_Y','Distance','AoA_azimuth'])
    
    unlab = Unlab_SR150_Resp()
    unlab.Positioning()