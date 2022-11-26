import time
import datetime

from colorama import Fore

import serial
import serial.tools.list_ports
import serial.serialutil

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

def time_ms():
    return round(datetime.datetime.utcnow().timestamp() * 1000)

def Put_serial(q, COM_PORT):
    ## Reset all ##
    delay = 3
    cnt_s = 0
    scpi_rx = serial.Serial(COM_PORT, baudrate=230400, timeout=6)
    
    state_ntf_rx = serial_trx(scpi_rx, "RST\r\n") # 1. 'RST' Command to board(TX) 2. Read response from board(RX)
    print(state_ntf_rx)
    time.sleep(delay)
    
    ## Ranging start ##
    state_ntf_rx = serial_trx(scpi_rx, "UWB MTRESP ON\r\n") # 'Session start' Command
    print(state_ntf_rx)
    time.sleep(delay)

    while True: 
        scpi_rx.flush() # Wait until all data is written
        scpi_ret = serial_rx(scpi_rx)
        scpi_ret += str(time_ms())
        
        if q.qsize() < 13 and cnt_s == 0:
            q.put(scpi_ret)

        elif q.qsize() >= 13 or cnt_s != 0 :
            print(Fore.GREEN,q.qsize(),Fore.RESET)
            cnt_s += 1
            if cnt_s % 8 == 0 :
                cnt_s = 0 
                