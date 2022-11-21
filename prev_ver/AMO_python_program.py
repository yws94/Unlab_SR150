import csv
import os
import random
import signal
import socket
import threading
import time
import numpy
import pyvisa as visa
from colorama import Fore


import serial
import serial.tools.list_ports
import serial.serialutil
import datetime



# ---------------------------------TEST RUN CONFIGS---------------------------------------------------------------------
TEST_OBJECT_FW = False  # make this false for test object, True for release FW(RC)

Rx_DEVICE_COM_PORT = 'com30' #responder COM Port
Tx1_DEVICE_COM_PORT = 'com31' #initiator1 COM Port
Tx2_DEVICE_COM_PORT = 'com32' #initiator2 COM Port
Tx3_DEVICE_COM_PORT = 'com34' #initiator3 COM Port 
Tx4_DEVICE_COM_PORT = 'com35' #initiator4 COM Port

# ----------------------------------------------------------------------------------------------------------------------
# --------------------------------------NO EDITS BELOW------------------------------------------------------------------

# ----------------------------------------------------------------------------------------------------------------------


# scpi = serial.Serial(DEVICE_COM_PORT, baudrate=115200, timeout=6)
# scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)
# scpi_tx1 = serial.Serial(Tx1_DEVICE_COM_PORT, baudrate=230400, timeout=6)
# scpi_tx2 = serial.Serial(Tx2_DEVICE_COM_PORT, baudrate=230400, timeout=6)
# scpi_tx3 = serial.Serial(Tx3_DEVICE_COM_PORT, baudrate=230400, timeout=6)
# scpi_tx4 = serial.Serial(Tx4_DEVICE_COM_PORT, baudrate=230400, timeout=6)
# scpi_initiator = serial.Serial(Initiator_DEVICE_COM_PORT, baudrate=115200, timeout=6)
# scpi_responder = serial.Serial(Responder_DEVICE_COM_PORT, baudrate=115200, timeout=6)

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

def UWB_SR100_Ranging_test():

    now = datetime.datetime.now()
    nowDatetime = now.strftime('%Y_%m_%d_%H_%M_%S')

    ranging_result_csvF = 'UWB_SR100_ranging_test_result-%s.csv' %nowDatetime
    save_csv(ranging_result_csvF, ['Session_ID','Distance','AoA_azimuth','AoA_elevation','PDoA_azimuth','PDoA_elevation'])

    reset_delay = 3 # delay setting(sec)
    ranging_stop_delay = 1 # delay setting(Sec)
    delay = 1 # delay setting(sec)
    test_count = 0
    Data_index = 0
    num = 3 # number of data
    
    while test_count<3:
        ## set baudrate ##
        scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6) 
        scpi_tx1 = serial.Serial(Tx1_DEVICE_COM_PORT, baudrate=230400, timeout=6) 
        scpi_tx2 = serial.Serial(Tx2_DEVICE_COM_PORT, baudrate=230400, timeout=6) 
        scpi_tx3 = serial.Serial(Tx3_DEVICE_COM_PORT, baudrate=230400, timeout=6) 
        scpi_tx4 = serial.Serial(Tx4_DEVICE_COM_PORT, baudrate=230400, timeout=6) 

        ## Reset all ##
        state_ntf_tx = serial_trx(scpi_tx1, "RST\r\n") # 'reset' command
        print(state_ntf_tx)
        state_ntf_tx = serial_trx(scpi_tx2, "RST\r\n")
        print(state_ntf_tx)
        state_ntf_tx = serial_trx(scpi_tx3, "RST\r\n")
        print(state_ntf_tx)
        state_ntf_tx = serial_trx(scpi_tx4, "RST\r\n")
        print(state_ntf_tx)
        state_ntf_rx = serial_trx(scpi_rx, "RST\r\n")
        print(state_ntf_rx)
        time.sleep(reset_delay)

        state_ntf_tx = serial_rx(scpi_tx1)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(scpi_tx2)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(scpi_tx3)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(scpi_tx4)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(scpi_rx)
        print(state_ntf_tx)

        ## Session #1 Ranging start ##
        state_ntf_tx = serial_trx(scpi_tx1, "UWB MTINIT1 ON\r\n") # Initiator of Session #1 start Command
        print(state_ntf_tx) 
        state_ntf_rx = serial_trx(scpi_rx, "UWB MTRESP1 ON\r\n") # Responder of Session #1 start Command
        print(state_ntf_rx)
        state_ntf_tx = serial_rx(scpi_tx1)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(scpi_rx)
        print(state_ntf_tx)
        time.sleep(delay)

        scpi_tx1.close()
        scpi_rx.close()

        while Data_index< num: # Number of result Data
            scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)
            scpi_ret = serial_rx(scpi_rx)
            print(Fore.GREEN,scpi_ret,Fore.RESET)
            try:
                ## Data Parsing ##
                result = scpi_ret.split(',')
                session_id = result[0]
                distance = result[1]
                aoa_azimuth = result[2]
                aoa_elevation = result[3]
                pdoa_azimuth = result[4]
                pdoa_elevation = result[5].replace("\r\n", '')
            except:
                pass
            ## save data(.csv file) ##
            save_csv(ranging_result_csvF, [session_id, distance, aoa_azimuth, aoa_elevation, pdoa_azimuth, pdoa_elevation])
            Data_index = Data_index + 1
            scpi_rx.close()
        
        scpi_tx1 = serial.Serial(Tx1_DEVICE_COM_PORT, baudrate=230400, timeout=6)
        scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)

        state_ntf_rx = serial_trx(scpi_rx, "UWB RNG STOP\r\n") #Reset Command
        print(state_ntf_rx)
        state_ntf_tx = serial_trx(scpi_tx1, "UWB RNG STOP\r\n") #Reset Command
        print(state_ntf_tx)
        time.sleep(ranging_stop_delay)
        state_ntf_tx = serial_rx(scpi_tx1)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(scpi_rx)
        print(state_ntf_tx)

        state_ntf_tx = serial_trx(scpi_tx2, "UWB MTINIT2 ON\r\n") #Initiator of Session #2 ON Command
        print(state_ntf_tx)
        state_ntf_rx = serial_trx(scpi_rx, "UWB MTRESP2 ON\r\n") #Responder of Session #2 ON Command
        print(state_ntf_rx)
        state_ntf_tx = serial_rx(scpi_tx2)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(scpi_rx)
        print(state_ntf_tx)
        time.sleep(delay)
        scpi_tx2.close()
        scpi_rx.close()

        Data_index=0
        while Data_index< num:# Number of result Data
            scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)
            scpi_ret = serial_rx(scpi_rx)
            print(Fore.GREEN,scpi_ret,Fore.RESET)
            try:
                ## Data Parsing ##
                result = scpi_ret.split(',')
                session_id = result[0]
                distance = result[1]
                aoa_azimuth = result[2]
                aoa_elevation = result[3]
                pdoa_azimuth = result[4]
                pdoa_elevation = result[5].replace("\r\n",'')
            except:
                pass
            ## Save Data ##
            save_csv(ranging_result_csvF, [session_id,distance,aoa_azimuth,aoa_elevation,pdoa_azimuth,pdoa_elevation])
            Data_index = Data_index + 1
            scpi_rx.close()
        
        scpi_tx2 = serial.Serial(Tx2_DEVICE_COM_PORT, baudrate=230400, timeout=6)
        scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)

        state_ntf_rx = serial_trx(scpi_rx, "UWB RNG STOP\r\n") #Reset Command
        print(state_ntf_rx)
        state_ntf_tx = serial_trx(scpi_tx2, "UWB RNG STOP\r\n") #Reset command
        print(state_ntf_tx)
        time.sleep(ranging_stop_delay)
        state_ntf_tx = serial_rx(scpi_tx2)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(scpi_rx)
        print(state_ntf_tx)
        
        state_ntf_tx = serial_trx(scpi_tx3, "UWB MTINIT3 ON\r\n") #Initiator of Session #3 ON Command
        print(state_ntf_tx)
        state_ntf_rx = serial_trx(scpi_rx, "UWB MTRESP3 ON\r\n") #Responder of Session #3 ON Command
        print(state_ntf_rx)
        state_ntf_tx = serial_rx(scpi_tx3)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(scpi_rx)
        print(state_ntf_tx)
        time.sleep(delay)
        scpi_tx3.close()
        scpi_rx.close()

        Data_index=0
        while Data_index< num:# Number of result Data
            scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)
            scpi_ret = serial_rx(scpi_rx)
            print(Fore.GREEN,scpi_ret,Fore.RESET)
            try:
                ## Data Parsing ##
                result = scpi_ret.split(',')
                session_id = result[0]
                distance = result[1]
                aoa_azimuth = result[2]
                aoa_elevation = result[3]
                pdoa_azimuth = result[4]
                pdoa_elevation = result[5].replace("\r\n", '')
            except:
                pass
            ## Save Data ##
            save_csv(ranging_result_csvF, [session_id, distance, aoa_azimuth, aoa_elevation, pdoa_azimuth, pdoa_elevation])
            Data_index = Data_index + 1
            scpi_rx.close()
        
        scpi_tx3 = serial.Serial(Tx3_DEVICE_COM_PORT, baudrate=230400, timeout=6)
        scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)

        state_ntf_rx = serial_trx(scpi_rx, "UWB RNG STOP\r\n") #Reset Command
        print(state_ntf_rx)
        state_ntf_tx = serial_trx(scpi_tx3, "UWB RNG STOP\r\n") #Reset Command
        print(state_ntf_tx)
        time.sleep(ranging_stop_delay)
        state_ntf_tx = serial_rx(scpi_tx3)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(scpi_rx)
        print(state_ntf_tx)
        
        state_ntf_tx = serial_trx(scpi_tx4, "UWB MTINIT4 ON\r\n") #Initiator of Session #4 ON Command
        print(state_ntf_tx)
        state_ntf_rx = serial_trx(scpi_rx, "UWB MTRESP4 ON\r\n") #Responder of Session #4 ON Command
        print(state_ntf_rx)
        state_ntf_tx = serial_rx(scpi_tx4)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(scpi_rx)
        print(state_ntf_tx)
        time.sleep(delay)
        scpi_tx4.close()
        scpi_rx.close()

        Data_index=0
        while Data_index< num:# Number of result Data
            scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)
            scpi_ret = serial_rx(scpi_rx)
            print(Fore.GREEN,scpi_ret,Fore.RESET)
            try:
                ## Data Parsing ##
                result = scpi_ret.split(',')
                session_id = result[0]
                distance = result[1]
                aoa_azimuth = result[2]
                aoa_elevation = result[3]
                pdoa_azimuth = result[4]
                pdoa_elevation = result[5].replace("\r\n", '')
            except:
                pass
            ## Save Data ##
            save_csv(ranging_result_csvF, [session_id, distance, aoa_azimuth, aoa_elevation, pdoa_azimuth, pdoa_elevation])
            Data_index = Data_index + 1
            scpi_rx.close()
        
        scpi_tx4 = serial.Serial(Tx4_DEVICE_COM_PORT, baudrate=230400, timeout=6)
        scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)

        state_ntf_rx = serial_trx(scpi_rx, "UWB RNG STOP\r\n") #Reset Command
        print(state_ntf_rx)
        state_ntf_tx = serial_trx(scpi_tx4, "UWB RNG STOP\r\n") #Reset Command
        print(state_ntf_tx)
        time.sleep(ranging_stop_delay)
        state_ntf_tx = serial_rx(scpi_tx4)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(scpi_rx)
        print(state_ntf_tx)

        scpi_rx.close()
        scpi_tx1.close()
        scpi_tx2.close()
        scpi_tx3.close()
        scpi_tx4.close()

        test_count=test_count+1

UWB_SR100_Ranging_test()
