import os
import csv
import datetime
from fxpmath import Fxp
from multiprocessing import Process, Queue

import numpy as np
import math

from colorama import Fore

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import tensorflow as tf

from Sock import *
from Read_Serial import *
from Parse_DNN_ver2 import *
from AKF import *
from Correct_pos_ver4 import *

# ----------------------------------------------------------------------------------------------------------------------
# --------------------------------------NO EDITS BELOW------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

def load_csv():
    now = datetime.datetime.now()
    nowDatetime = now.strftime('%Y_%m_%d_%H_%M_%S')
    csvF = 'Positioning_test_result-%s.csv' %nowDatetime
    save_csv(csvF, ['TARGET_X','TARGET_Y','POS1_X','POS1_Y','POS2_X','POS2_Y','POS3_X','POS3_Y','POS4_X','POS4_Y'])
    return csvF

def save_csv(csvf, row):
    with open(csvf, "a", newline="") as F:
        w = csv.writer(F)
        w.writerow(row)

def calc_nlos(p_dnn):
    model = tf.keras.models.load_model('HH01.h5')
    input_dnn = np.array([Parsing_DNN(p_dnn)])
    y_predict = model.predict(x=input_dnn)
    return y_predict[0][0]
    
# ----------------------------------------------------------------------------------------------------------------------

class Positioning():
    def __init__(self):
        self.KF = AKF()
        self.Corr = Correction(dist_x = 0.5, dist_y = 0.6)
        self.h_diff = 1.8 - 0.8
        self.check_dt = 0
        self.outlier = 1.0
        self.x1, self.y1, self.x2, self.y2, self.x3, self.y3, self.x4, self.y4 = 0,0,0,0,0,0,0,0
        
    # def Parsing(self, q1, q2, csv):    
    def Parsing(self, q1, q2):
        last_time = 0
        selected = []
        
        while q1:
            try:
                q_data = q1.get()
                result = list(q_data.split(' '))

                id = result[1]
                nlos = calc_nlos(result)

                distance = Fxp(val="0x"+result[5]+"0x"+result[4], signed=False, n_word=16, n_frac=0).astype(int).tolist() - 10
                dist = math.sqrt(math.pow(float(distance)/100,2) - math.pow(self.h_diff,2))
                # dist = float(distance)/100
                
                AoA_azimuth = Fxp(val="0x"+result[7]+"0x"+result[6], signed=True, n_word=16, n_frac=7).astype(float)
                AoA = float(AoA_azimuth)
                angle = math.pi * (AoA+90)/180
                
                curr_time = int(result[366][-7:])
                dt = curr_time - last_time
                self.check_dt += dt

                ## Calculate Positions of TAGs
                x_raw, y_raw = dist * math.cos(angle), dist * math.sin(angle)
                meas = np.array([[x_raw],[y_raw]])

                if id == '11':
                    r1Temp, r2Temp = self.KF.r1 + self.KF.r1_gap * nlos, self.KF.r2 + self.KF.r2_gap * nlos
                    self.KF.R1 = np.diag([r1Temp , r2Temp])
                    check_r = math.sqrt(math.pow((x_raw - self.x1),2) + math.pow((y_raw - self.y1),2))
                    
                    if check_r < self.outlier:
                        self.KF.kf_update1(meas)
                    else :
                        new_x, new_y = (x_raw + self.KF.X1[0][0])/2, (y_raw + self.KF.X1[1][0])/2
                        meas = np.array([[new_x],[new_y]])
                        self.KF.kf_update1(meas)
                        
                    self.KF.fst1, kf_x1, kf_y1 = False, round(self.KF.X1[0][0],3), round(self.KF.X1[1][0],3)
                    self.x1, self.y1 = x_raw, y_raw
                
                elif id == '22':
                    r1Temp, r2Temp = self.KF.r1 + self.KF.r1_gap * nlos, self.KF.r2 + self.KF.r2_gap * nlos
                    self.KF.R2 = np.diag([r1Temp , r2Temp])
                    check_r = math.sqrt(math.pow((x_raw - self.x2),2) + math.pow((y_raw - self.y2),2))
                    
                    if check_r < self.outlier:
                        self.KF.kf_update1(meas)
                    else :
                        new_x, new_y = (x_raw + self.KF.X2[0][0])/2, (y_raw + self.KF.X2[1][0])/2
                        meas = np.array([[new_x],[new_y]])
                        self.KF.kf_update2(meas)
                        
                    self.KF.fst2, kf_x2, kf_y2 = False, round(self.KF.X2[0][0],3), round(self.KF.X2[1][0],3)
                    self.x2, self.y2 = x_raw, y_raw
                    
                elif id == '33':
                    r1Temp, r2Temp = self.KF.r1 + self.KF.r1_gap * nlos, self.KF.r2 + self.KF.r2_gap * nlos
                    self.KF.R3 = np.diag([r1Temp , r2Temp]) 
                    check_r = math.sqrt(math.pow((x_raw - self.x3),2) + math.pow((y_raw - self.y3),2))
                    
                    if check_r < self.outlier:
                        self.KF.kf_update1(meas)
                    else :
                        new_x, new_y = (x_raw + self.KF.X3[0][0])/2, (y_raw + self.KF.X3[1][0])/2
                        meas = np.array([[new_x],[new_y]])
                        self.KF.kf_update3(meas)
                        
                    self.KF.fst3, kf_x3, kf_y3 = False, round(self.KF.X3[0][0],3), round(self.KF.X3[1][0],3)
                    self.x3, self.y3 = x_raw, y_raw
                    
                elif id == '44':
                    r1Temp, r2Temp = self.KF.r1 + self.KF.r1_gap * nlos, self.KF.r2 + self.KF.r2_gap * nlos
                    self.KF.R4 = np.diag([r1Temp , r2Temp]) 
                    check_r = math.sqrt(math.pow((x_raw - self.x4),2) + math.pow((y_raw - self.y4),2))
                    
                    if check_r < self.outlier:
                        self.KF.kf_update1(meas)
                    else :
                        new_x, new_y = (x_raw + self.KF.X4[0][0])/2, (y_raw + self.KF.X4[1][0])/2
                        meas = np.array([[new_x],[new_y]])
                        self.KF.kf_update4(meas)

                    self.KF.fst4, kf_x4, kf_y4 = False, round(self.KF.X4[0][0],3), round(self.KF.X4[1][0],3)
                    self.x4, self.y4 = x_raw, y_raw
                    
                if self.check_dt < 370:
                    if id == '11': selected.append([id,kf_x1,kf_y1,nlos])
                    elif id == '22': selected.append([id,kf_x2,kf_y2,nlos])
                    elif id == '33': selected.append([id,kf_x3,kf_y3,nlos])
                    elif id == '44': selected.append([id,kf_x4,kf_y4,nlos])
                    
                else:
                    target, pos1, pos2, pos3, pos4 = self.Corr.corr_pos(selected)
                    
                    if target != False :
                        q2.put(target)
                        # save_csv(csv, [target[0],target[1],pos1[0],pos1[1],pos2[0],pos2[1],pos3[0],pos3[1],pos4[0],pos4[1]])
                    selected.clear()
                    self.check_dt = 0
                    
                    if id == '11': selected.append([id,kf_x1,kf_y1,nlos])
                    elif id == '22': selected.append([id,kf_x2,kf_y2,nlos])
                    elif id == '33': selected.append([id,kf_x3,kf_y3,nlos])
                    elif id == '44': selected.append([id,kf_x4,kf_y4,nlos])
                
                print(id, Fore.RED, dt, Fore.RESET)
                last_time = curr_time
                result.clear()
            except : pass
    
if __name__ == "__main__": 
    server_ip, port_num = '166.104.46.232', 8080
    Rx_DEVICE_COM_PORT = 'com16' #responder COM Port
    
    # f_name = load_csv()
    p = Positioning()
    q1, q2 = Queue(), Queue()

    proc1 = Process(target=Put_serial, args = (q1, Rx_DEVICE_COM_PORT))
    # p2 = Process(target=p.Parsing, args=(q1, q2, f_name))
    proc2 = Process(target=p.Parsing, args=(q1,q2))
    
    serv = AppServer(ip=server_ip, port=port_num, p1=proc1, p2=proc2, q2=q2)
    serv.Connect()
