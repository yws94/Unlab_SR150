import os, sys
from re import L
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
from multiprocessing import Process, Queue

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
# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

def put_serial(q):
    ## Reset all ##
    delay = 3
    scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)
    state_ntf_rx = serial_trx(scpi_rx, "RST\r\n")
    print(state_ntf_rx)
    time.sleep(delay)
    
    ## Ranging start ##
    state_ntf_rx = serial_trx(scpi_rx, "UWB MTRESP ON\r\n") # Responder Session start Command
    print(state_ntf_rx)
    time.sleep(delay)

    while True: 
        scpi_ret = serial_rx(scpi_rx)
        q.put(scpi_ret)
        scpi_rx.flush()


class Positioning():
    def __init__(self):
        self.ekf = EKF_AoA()

    def calc_ekf(self, e_data):
        id = e_data[0]
        x,y = e_data[1], e_data[2]
        meas = np.array([[x],[y]])
        
        if id == '11':
            self.ekf.ekf_update1(meas)
            self.ekf.cnt1 = 1
            print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,Fore.GREEN,"TAG 1 EKF : ({:.2f}, {:.2f})".format(self.ekf.X1[0][0],self.ekf.X1[1][0]),"\n",Fore.RESET)
        
        elif id == '22':
            self.ekf.ekf_update2(meas)
            self.ekf.cnt2 = 1
            print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,"TAG 2 EKF : ({:.2f}, {:.2f})".format(self.ekf.X2[0][0],self.ekf.X2[1][0]),"\n")
            
        elif id == '33':
            self.ekf.ekf_update3(meas)
            self.ekf.cnt3 = 1
            print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,"TAG 3 EKF : ({:.2f}, {:.2f})".format(self.ekf.X3[0][0],self.ekf.X3[1][0]),"\n")
            
        elif id == '44':
            self.ekf.ekf_update4(meas)
            self.ekf.cnt4 = 1
            print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,"TAG 4 EKF : ({:.2f}, {:.2f})".format(self.ekf.X4[0][0],self.ekf.X4[1][0]),"\n")
            
        else : pass
    
    def parsing(self, q):
        while True:
            try:
                q_data = q.get()
                result = q_data.split(' ')
                session_id = result[0]
                seq = Fxp(val="0x"+result[3]+"0x"+result[2], signed=False, n_word=16, n_frac=0).astype(int).tolist()
                distance = Fxp(val="0x"+result[5]+"0x"+result[4], signed=False, n_word=16, n_frac=0).astype(int).tolist() - 10
                AoA_azimuth = Fxp(val="0x"+result[7]+"0x"+result[6], signed=True, n_word=16, n_frac=7).astype(float)
                
                # input_dnn = Parsing_DNN(result)
                
                dist = float(distance)/100
                angle = math.pi * (float(AoA_azimuth)+90)/180
                
                # ## calculate position of TAGs
                x_raw = dist * math.cos(angle)
                y_raw = dist * math.sin(angle)
                ## convert types for dist and angle 
                # dist = math.sqrt(math.pow(float(distance)/100,2) - math.pow(self.h_diff,2))
                
                ekf_data = [session_id, x_raw, y_raw]
                self.calc_ekf(ekf_data)
                
                # self.plot()
                result.clear()
                # input_dnn.clear()    
                ekf_data.clear()
            except : pass

if __name__ == "__main__": 
    # now = datetime.datetime.now()
    # nowDatetime = now.strftime('%Y_%m_%d_%H_%M_%S')
    p = Positioning()
    q = Queue()

    p1 = Process(target=put_serial, args =(q,))
    p2 = Process(target=p.parsing, args=(q,))
    
    p1.start()
    p2.start()
    # ranging_result_csvF = 'results/UWB_SR150_ranging_test_result-%s.csv' %nowDatetime
    # save_csv(ranging_result_csvF, ['Session_ID','Distance','pos_X','pos_Y','ref_X','ref_Y','AoA_azimuth','PDoA_azimuth'])
    # save_csv(ranging_result_csvF, ['Session_ID','Distance','pos_X','pos_Y','ref_X','ref_Y','AoA_azimuth','PDoA_azimuth', 'Estimated_Err', 'Ref_Err'])