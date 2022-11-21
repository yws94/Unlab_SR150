import csv
import time
import datetime
from fxpmath import Fxp
from Parse_DNN_ver2 import *
from multiprocessing import Process, Queue

import math
from math import *

import serial
import serial.tools.list_ports
import serial.serialutil

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

def Modeling(q, csv):
    h_diff = 1.8 - 0.8
    while True:
        try:
            q_data = q.get()
            result = list(q_data.split(' '))
            id = result[1]
            distance = Fxp(val="0x"+result[5]+"0x"+result[4], signed=False, n_word=16, n_frac=0).astype(int).tolist() - 10
            AoA_azimuth = Fxp(val="0x"+result[7]+"0x"+result[6], signed=True, n_word=16, n_frac=7).astype(float)
            nlos = int(result[10])
                            
            dist = float(distance)/100
            # dist = math.sqrt(math.pow(float(distance)/100,2) - math.pow(h_diff,2))
            
            angle = math.pi * (float(AoA_azimuth)+90)/180

            input_dnn = Parsing_DNN(result)

            data = []
            data.append(id)
            data.append(nlos)
            data.append(dist)
            data.append(AoA_azimuth)

            save_csv(csv, data + input_dnn)

            result.clear()
            input_dnn.clear()
            data.clear()    
        except : pass

if __name__ == "__main__": 
    now = datetime.datetime.now()
    nowDatetime = now.strftime('%Y_%m_%d_%H_%M_%S')
    csvF = 'dnn_result-%s.csv' %nowDatetime
    
    save_csv(csvF, ['id','nlos','Distance','AoA','SNR main path 1','SNR main path 2','SNR main path 3','SNR main path 4','SNR first path 1','SNR first path 2','SNR first path 3','SNR first path 4',
                    'SNR Total 1','SNR Total 2','SNR Total 3', 'SNR Total 4','RSSI 1','RSSI 2','RSSI 3','RSSI 4','CIR main power 1','CIR main power 2', 'CIR main power 3','CIR main power 4',
                    'CIR first path power 1','CIR first path power 2','CIR first path power3','CIR first path power 4','Noise variance 1','Noise variance 2','Noise variance 3','Noise variance 4',
                    'CFO 1','CFO 2','CFO 3','CFO 4','AoA Phase 1','AoA Phase 2','AoA Phase 3','AoA Phase 4','First path index 1','First path index 2','First path index 3','First path index 4',
                    'Main path index 1','Main path index 2','Main path index 3','Main path index 4','Mapping 1','Mapping 2','Mapping 3','Mapping 4'])
    
    q = Queue()

    p1 = Process(target=put_serial, args =(q,))
    p2 = Process(target=Modeling, args=(q,csvF,))

    p1.start()
    p2.start()
   