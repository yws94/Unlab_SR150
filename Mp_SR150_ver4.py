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
from multiprocessing import Process, Queue

import numpy as np
import math
from math import *

import pyvisa as visa
from colorama import Fore

import serial
import serial.tools.list_ports
import serial.serialutil

import matplotlib.pyplot as plt

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import tensorflow as tf

from Parse_DNN_ver2 import *
from AKF_AoA import *
from Correct_pos_ver3 import *
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
    
def time_ms():
    return round(datetime.datetime.utcnow().timestamp() * 1000)

def calc_nlos(p_dnn):
    model = tf.keras.models.load_model('HH01.h5')
    input_dnn = np.array([Parsing_DNN(p_dnn)])
    y_predict = model.predict(x=input_dnn)
    return y_predict[0][0]
    
# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

def put_serial(q):
    ## Reset all ##
    delay = 3
    scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)
    state_ntf_rx = serial_trx(scpi_rx, "RST\r\n") # 1. 'RST' Command to board(TX) 2. Read response from board(RX)
    print(state_ntf_rx)
    time.sleep(delay)
    
    ## Ranging start ##
    state_ntf_rx = serial_trx(scpi_rx, "UWB MTRESP ON\r\n") # 'Session start' Command
    print(state_ntf_rx)
    time.sleep(delay)

    while True: 
        scpi_ret = serial_rx(scpi_rx)
        scpi_ret += str(time_ms())
        q.put(scpi_ret)
        scpi_rx.flush() # Wait until all data is written

class Positioning():
    def __init__(self):
        self.kf = AKF()
        self.corr = Corr()
        self.h_diff = 1.8 - 0.8
        self.check_dt = 0
    
    def parsing(self, q, csv):    
    # def parsing(self, q):
        last_time = 0
        selected = []
        while q:
            try:
                q_data = q.get()
                result = list(q_data.split(' '))
                
                id = result[1]
                nlos = calc_nlos(result)
                
                distance = Fxp(val="0x"+result[5]+"0x"+result[4], signed=False, n_word=16, n_frac=0).astype(int).tolist() - 10
                # dist = math.sqrt(math.pow(float(distance)/100,2) - math.pow(self.h_diff,2))
                dist = float(distance)/100
                
                AoA_azimuth = Fxp(val="0x"+result[7]+"0x"+result[6], signed=True, n_word=16, n_frac=7).astype(float)
                AoA = float(AoA_azimuth)
                angle = math.pi * (AoA+90)/180
                
                time = int(result[366][-7:])
                dt = time - last_time
                self.check_dt += dt

                ## Calculate Positions of TAGs
                x_raw, y_raw = dist * math.cos(angle), dist * math.sin(angle)
                meas = np.array([[x_raw],[y_raw]])

                if id == '11':
                    r1Temp, r2Temp = self.kf.r1 + self.kf.r1_gap * nlos, self.kf.r2 + self.kf.r2_gap * nlos
                    self.kf.R1 = np.diag([r1Temp , r2Temp])
                    self.kf.kf_update1(meas)
                    self.kf.cnt1, kf_x1, kf_y1 = 1, round(self.kf.X1[0][0],3), round(self.kf.X1[1][0],3)
                
                elif id == '22':
                    r1Temp, r2Temp = self.kf.r1 + self.kf.r1_gap * nlos, self.kf.r2 + self.kf.r2_gap * nlos
                    self.kf.R2 = np.diag([r1Temp , r2Temp]) 
                    self.kf.kf_update2(meas)
                    self.kf.cnt2, kf_x2, kf_y2 = 1, round(self.kf.X2[0][0],3), round(self.kf.X2[1][0],3)
                    
                elif id == '33':
                    r1Temp, r2Temp = self.kf.r1 + self.kf.r1_gap * nlos, self.kf.r2 + self.kf.r2_gap * nlos
                    self.kf.R3 = np.diag([r1Temp , r2Temp]) 
                    self.kf.kf_update3(meas)
                    self.kf.cnt3, kf_x3, kf_y3 = 1, round(self.kf.X3[0][0],3), round(self.kf.X3[1][0],3)
                    
                elif id == '44':
                    r1Temp, r2Temp = self.kf.r1 + self.kf.r1_gap * nlos, self.kf.r2 + self.kf.r2_gap * nlos
                    self.kf.R4 = np.diag([r1Temp , r2Temp]) 
                    self.kf.kf_update4(meas)
                    self.kf.cnt4, kf_x4, kf_y4 = 1, round(self.kf.X4[0][0],3), round(self.kf.X4[1][0],3)
                
                print(id, Fore.RED,dt,Fore.RESET)
                if self.check_dt < 360:
                    if id == '11': selected.append([id,kf_x1,kf_y1,nlos])
                    elif id == '22': selected.append([id,kf_x2,kf_y2,nlos])
                    elif id == '33': selected.append([id,kf_x3,kf_y3,nlos])
                    elif id == '44': selected.append([id,kf_x4,kf_y4,nlos])
                    
                else:
                    # target, ref, pos1, pos2, pos3, pos4, fst, snd = self.corr.corr_pos(selected))
                    target, ref, pos1, pos2, pos3, pos4 = self.corr.corr_pos(selected)
                    
                    if target != False :
                        # save_csv(csv, [target[0],target[1],ref[0],ref[1],pos1[0],pos1[1],pos2[0],pos2[1],pos3[0],pos3[1],pos4[0],pos4[1],fst,snd])
                        save_csv(csv, [target[0],target[1],ref[0],ref[1],pos1[0],pos1[1],pos2[0],pos2[1],pos3[0],pos3[1],pos4[0],pos4[1]])
                    selected.clear()
                    self.check_dt = 0
                    
                    if id == '11': selected.append([id,kf_x1,kf_y1,nlos])
                    elif id == '22': selected.append([id,kf_x2,kf_y2,nlos])
                    elif id == '33': selected.append([id,kf_x3,kf_y3,nlos])
                    elif id == '44': selected.append([id,kf_x4,kf_y4,nlos])

                last_time = time
                result.clear()
            except : pass

if __name__ == "__main__": 
    
    now = datetime.datetime.now()
    nowDatetime = now.strftime('%Y_%m_%d_%H_%M_%S')
    csvF = 'Positioning_test_result-%s.csv' %nowDatetime
    # save_csv(csvF, ['TARGET_X','TARGET_Y','REF_X','REF_Y','POS1_X','POS1_Y','POS2_X','POS2_Y','POS3_X','POS3_Y','POS4_X','POS4_Y','First Node','Second Node'])
    save_csv(csvF, ['TARGET_X','TARGET_Y','REF_X','REF_Y','POS1_X','POS1_Y','POS2_X','POS2_Y','POS3_X','POS3_Y','POS4_X','POS4_Y'])
    
    p = Positioning()
    q = Queue()

    p1 = Process(target=put_serial, args =(q,))
    p2 = Process(target=p.parsing, args=(q,csvF))
    # p2 = Process(target=p.parsing, args=(q,))
    
    p1.start()
    p2.start()