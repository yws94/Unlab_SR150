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
from EKF_AoA import EKF

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
        
class Unlab_SR150_Resp():
    def __init__(self):
        self.reset_delay = 3 # delay setting(sec)
        self.scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)
    
    def Parsing_DNN(self, result):
        DNN_data = []
        DNN_data.append(self.session_id)
                
        SNR_mp = [] # SNR Main Path
        SNR_fp = [] # SNR First Path
        SNR_Total = [] # SNR Total
        RSSI = [] 
        CIR_mp = [] #CIR main power
        CIR_fpp = [] #CIR firts path power
        Nv = [] #Noise variance
        CFO = []
        AoA_Phase = []
        Fp_index = [] # First Path index
        Mp_index = [] # Main path index
        Mapping = []
        CIR_sample_real_imag = [] #CIR sample 
        
        for i in range(0, 4):
            SNR_mp.append(Fxp(val="0x"+result[11+i*89], signed = False, n_word=8, n_frac = 0).astype(int).tolist())
            SNR_fp.append(Fxp(val="0x"+result[12+i*89], signed = False, n_word=8, n_frac = 0).astype(int).tolist())
            SNR_Total.append(Fxp(val="0x"+result[14+i*89]+"0x"+result[13+i*89], signed=False, n_word=16, n_frac=8).astype(float))
            RSSI.append(Fxp(val="0x"+result[16+i*89]+"0x"+result[15+i*89], signed=True, n_word=16, n_frac=8).astype(float))
            CIR_mp.append(Fxp(val="0x"+result[20+i*89]+"0x"+result[19+i*89]+"0x"+result[18+i*89]+"0x"+result[17+i*89], signed=False, n_word=32, n_frac=0).astype(int).tolist())
            CIR_fpp.append(Fxp(val="0x"+result[24+i*89]+"0x"+result[23+i*89]+"0x"+result[22+i*89]+"0x"+result[21+i*89], signed=False, n_word=32, n_frac=0).astype(int).tolist())
            Nv.append(Fxp(val="0x"+result[26+i*89]+"0x"+result[25+i*89], signed=False, n_word=16, n_frac=0).astype(int).tolist())    
            CFO.append(Fxp(val="0x"+result[28+i*89]+"0x"+result[27+i*89], signed=True, n_word=16, n_frac=0).astype(int).tolist())
            AoA_Phase.append(Fxp(val="0x"+result[30+i*89]+"0x"+result[29+i*89], signed=False, n_word=16, n_frac=7).astype(float))   
            Fp_index.append(Fxp(val="0x"+result[32+i*89]+"0x"+result[31+i*89], signed=False, n_word=16, n_frac=6).astype(float))  
            Mp_index.append(Fxp(val="0x"+result[34+i*89]+"0x"+result[33+i*89], signed=False, n_word=16, n_frac=6).astype(float))  
            Mapping.append(Fxp(val="0x"+result[35+i*89], signed = False, n_word=8, n_frac = 0).astype(int).tolist())
            
        DNN_data += SNR_mp + SNR_fp + SNR_Total + RSSI + CIR_mp + Nv + CFO + AoA_Phase + Fp_index + Mp_index + Mapping

        for i in range(0,32):
            CIR_sample_real_imag.append(Fxp(val="0x"+result[37+i*2]+"0x"+result[36+i*2], signed=False, n_word=16, n_frac=0).astype(int).tolist())
        for i in range(0,32):
            CIR_sample_real_imag.append(Fxp(val="0x"+result[126+i*2]+"0x"+result[125+i*2], signed=False, n_word=16, n_frac=0).astype(int).tolist())
        for i in range(0,32):
            CIR_sample_real_imag.append(Fxp(val="0x"+result[215+i*2]+"0x"+result[214+i*2], signed=False, n_word=16, n_frac=0).astype(int).tolist())
        for i in range(0,32):
            CIR_sample_real_imag.append(Fxp(val="0x"+result[304+i*2]+"0x"+result[303+i*2], signed=False, n_word=16, n_frac=0).astype(int).tolist())
        
        DNN_data += CIR_sample_real_imag
        return DNN_data
        
    def Positioning(self):
        ekf_data = []
        DNN = []
        ## Reset all ##
        state_ntf_rx = serial_trx(self.scpi_rx, "RST\r\n")
        print(state_ntf_rx)
        time.sleep(self.reset_delay)

        # state_ntf_rx = serial_trx(self.scpi_rx, "UWB PDOAOFFSET -50\r\n") # Input offset value
        # time.sleep(self.reset_delay)
        # print(state_ntf_rx)
        
        state_ntf_rx = serial_trx(self.scpi_rx, "UWB MTRESP ON\r\n") # Responder of Session #1 start Command
        print(state_ntf_rx)
        time.sleep(self.reset_delay)
        
        while True: 
            self.scpi_ret = serial_rx(self.scpi_rx)

            ## EKF Data Parsing ##
            try:
                result = self.scpi_ret.split(' ')
                
                ## EKF Data Parsing ##
                self.session_id = result[0]
                distance = Fxp(val="0x"+result[5]+"0x"+result[4], signed=False, n_word=16, n_frac=0).astype(int).tolist() - 10
                AoA_azimuth = Fxp(val="0x"+result[7]+"0x"+result[6], signed=True, n_word=16, n_frac=7).astype(float)
                PDoA_azimuth = Fxp(val="0x"+result[9]+"0x"+result[8], signed=True, n_word=16, n_frac=7).astype(float)
                nlos = Fxp(val="0x"+result[10], signed = False, n_word=8, n_frac = 0).astype(int).tolist()
                
                ekf_data = [self.session_id, distance, AoA_azimuth, nlos]

                ## DNN Data Parsing ##
                # DNN = self.Parsing_DNN(result)
                
                ## convert types for dist and angle ##
                dist = float(distance)/100
                angle = math.pi * (float(AoA_azimuth)+90)/180
                # s_dist = str(dist)
                
                ## calculate position of TAGs ##
                x = dist * math.cos(angle)
                y = dist * math.sin(angle)
                # x_ref = str(x)
                # y_ref = str(y)
                meas = np.array([[x],[y]])
                # print("({:.2f}, {:.2f})".format(x,y))
                
                if (len(ekf_data) == 4) and (ekf_data[0] == '11'):
                    print("TAG 1 : ({:.2f}, {:.2f})".format(x,y),"\n")

                elif (len(ekf_data) == 4) and (ekf_data[0] == '22'):
                    print("TAG 2 : ({: .2f} , {: .2f})".format(x,y),"\n")

                elif (len(ekf_data) == 4) and (ekf_data[0] == '33'):     
                    print(Fore.GREEN,"TAG 3 : ({: .2f} , {: .2f})".format(x,y),"\n",Fore.RESET)
           
                elif (len(ekf_data) == 4) and (ekf_data[0] == '44'):     
                    print("TAG 4 : ({: .2f} , {: .2f})".format(x,y),"\n")

                else : pass  
            
            except:
                pass

                ekf_data.clear()
                DNN.clear()
                result.clear()
                self.scpi_rx.flush()

            # ## save data(.csv file) ##
            # save_csv(ranging_result_csvF, [session_id, s_dist, x_pos, y_pos, x_ref, y_ref,aoa_azimuth, pdoa_azimuth])
            # save_csv(ranging_result_csvF, [session_id, s_dist, x_pos, y_pos, x_ref, y_ref,aoa_azimuth, pdoa_azimuth, e_err, r_err])

            
if __name__ == "__main__": 
    # now = datetime.datetime.now()
    # nowDatetime = now.strftime('%Y_%m_%d_%H_%M_%S')
    
    # ranging_result_csvF = 'results/UWB_SR150_ranging_test_result-%s.csv' %nowDatetime
    # save_csv(ranging_result_csvF, ['Session_ID','Distance','pos_X','pos_Y','ref_X','ref_Y','AoA_azimuth','PDoA_azimuth'])
    # save_csv(ranging_result_csvF, ['Session_ID','Distance','pos_X','pos_Y','ref_X','ref_Y','AoA_azimuth','PDoA_azimuth', 'Estimated_Err', 'Ref_Err'])
    EKF1 = EKF()
    EKF2 = EKF()
    EKF3 = EKF()
    EKF4 = EKF()

    ekf = Unlab_SR150_Resp()
    ekf.Positioning()