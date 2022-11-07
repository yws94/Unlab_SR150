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
from Parse_DNN_ver2 import *
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
fin = 0 
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
    
def time_ms():
    return round(datetime.datetime.utcnow().timestamp() * 1000)

def corr_pos(tag_pos):
    n = len(tag_pos)
    pos_x, pos_y = 0, 0
    if 2 <= n <= 4:
        for i in range(n):
            pos_x += tag_pos[i][1]
            pos_y += tag_pos[i][2]
        return round(pos_x/n,3), round(pos_y/n,3), n
    else:
        return False, False, False

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
        self.h_diff = 1.8 - 0.8
        
    def parsing(self, q):
    # def parsing(self, q, csv):
        global fin
        selected = []
        while q:
            try:
                q_data = q.get()
                start = time_ms()

                result = list(q_data.split(' '))
                
                id = result[1]
                distance = Fxp(val="0x"+result[5]+"0x"+result[4], signed=False, n_word=16, n_frac=0).astype(int).tolist() - 10
                AoA_azimuth = Fxp(val="0x"+result[7]+"0x"+result[6], signed=True, n_word=16, n_frac=7).astype(float)
                nlos = int(result[10])
                
                # input_dnn = Parsing_DNN(result)
                
                AoA = float(AoA_azimuth)
                angle = math.pi * (AoA+90)/180
                
                # dist = math.sqrt(math.pow(float(distance)/100,2) - math.pow(self.h_diff,2))
                dist = float(distance)/100
                
                ## calculate position of TAGs
                x_raw = dist * math.cos(angle)
                y_raw = dist * math.sin(angle)

                meas = np.array([[x_raw],[y_raw]])

                if id == '11':
                    self.ekf.ekf_update1(meas)
                    self.ekf.cnt1 = 1
                    e_x1, e_y1 = round(self.ekf.X1[0][0],3), round(self.ekf.X1[1][0],3)
                    # print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,Fore.GREEN,"TAG 1 EKF : ({:.2f}, {:.2f})".format(e_x1, e_y1),"\n",Fore.RESET)
                    # save_csv(csv, ['',str(id), str(e_x1), str(e_y1), str(dist), str(-AoA)])
                
                elif id == '22':
                    self.ekf.ekf_update2(meas)
                    self.ekf.cnt2 = 1
                    e_x2, e_y2 = round(self.ekf.X2[0][0],3), round(self.ekf.X2[1][0],3)
                    # print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,"TAG 2 EKF : ({:.2f}, {:.2f})".format(e_x2, e_y2),"\n")
                    # save_csv(csv, ['',str(id), str(e_x2), str(e_y2), str(dist), str(-AoA)])
                    
                elif id == '33':
                    self.ekf.ekf_update3(meas)
                    self.ekf.cnt3 = 1
                    e_x3, e_y3 = round(self.ekf.X3[0][0],3), round(self.ekf.X3[1][0],3)
                    # print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,"TAG 3 EKF : ({:.2f}, {:.2f})".format(e_x3, e_y3),"\n")
                    # save_csv(csv, ['',str(id), str(e_x3), str(e_y3), str(dist), str(-AoA)])
                    
                elif id == '44':
                    self.ekf.ekf_update4(meas)
                    self.ekf.cnt4 = 1
                    e_x4, e_y4 = round(self.ekf.X4[0][0],3), round(self.ekf.X4[1][0],3)
                    # save_csv(csv, ['',str(id), str(e_x4), str(e_y4), str(dist), str(-AoA)])
                    # print(Fore.RED,datetime.datetime.now().time(),Fore.RESET,"TAG 4 EKF : ({:.2f}, {:.2f})".format(e_x4, e_y4),"\n")
                    
                if start - fin < 180:
                    if id == '11':
                        selected.append([id,e_x1,e_y1,nlos])
                    elif id == '22':
                        selected.append([id,e_x2,e_y2,nlos])
                    elif id == '33':
                        selected.append([id,e_x3,e_y3,nlos])
                    elif id == '44':
                        selected.append([id,e_x4,e_y4,nlos])
                else:
                    x,y,n = corr_pos(selected)
                    if x != False:
                        c_time = datetime.datetime.now().time()
                        print(Fore.RED,c_time,Fore.RESET,c_time,x,y,n,'\n') ## function ##
                        # save_csv(csv, [str(c_time),'total', str(x), str(y),str(n)])
                    
                    selected.clear()
                    
                    if id == '11':
                        selected.append([id,e_x1,e_y1,nlos])
                    elif id == '22':
                        selected.append([id,e_x2,e_y2,nlos])
                    elif id == '33':
                        selected.append([id,e_x3,e_y3,nlos])
                    elif id == '44':
                        selected.append([id,e_x4,e_y4,nlos])
                    
                fin = time_ms()
                result.clear()
                # input_dnn.clear()    
            except : pass

if __name__ == "__main__": 
    
    # now = datetime.datetime.now()
    # nowDatetime = now.strftime('%Y_%m_%d_%H_%M_%S')
    # csvF = 'UWB_SR150_ranging_test_result-%s.csv' %nowDatetime
    # save_csv(csvF, ['Time','TAG_ID','pos_X','pos_Y','Distance','AoA_azimuth'])
    
    p = Positioning()
    q = Queue()

    p1 = Process(target=put_serial, args =(q,))
    # p2 = Process(target=p.parsing, args=(q,csvF))
    p2 = Process(target=p.parsing, args=(q,))
    
    p1.start()
    p2.start()