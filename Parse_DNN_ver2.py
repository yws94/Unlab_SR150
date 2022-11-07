'''
Author : woosang
Date : 16.9.2022
Name : DNN_Data_Parsing
Language : Python 3.9.12
License: GPLv2 License
'''
from fxpmath import Fxp

def Parsing_DNN(result):
    DNN_data = []
            
    SNR_mp = [] # SNR Main Path
    SNR_fp = [] # SNR First Path
    SNR_Total = [] # SNR Total
    rssi = [] 
    CIR_mp = [] #CIR main power
    CIR_fpp = [] #CIR firts path power
    Nv = [] #Noise variance
    cfo = []
    AoA_Phase = []
    Fp_index = [] # First Path index
    Mp_index = [] # Main path index
    Mapping = []
    
    for i in range(0, 4):
        SNR_mp.append(Fxp(val="0x"+result[11+i*89], signed = False, n_word=8, n_frac = 0).astype(int).tolist())
        SNR_fp.append(Fxp(val="0x"+result[12+i*89], signed = False, n_word=8, n_frac = 0).astype(int).tolist())
        SNR_Total.append(Fxp(val="0x"+result[14+i*89]+"0x"+result[13+i*89], signed=False, n_word=16, n_frac=8).astype(float))
        rssi.append(Fxp(val="0x"+result[16+i*89]+"0x"+result[15+i*89], signed=True, n_word=16, n_frac=8).astype(float))
        CIR_mp.append(Fxp(val="0x"+result[20+i*89]+"0x"+result[19+i*89]+"0x"+result[18+i*89]+"0x"+result[17+i*89], signed=False, n_word=32, n_frac=0).astype(int).tolist())
        CIR_fpp.append(Fxp(val="0x"+result[24+i*89]+"0x"+result[23+i*89]+"0x"+result[22+i*89]+"0x"+result[21+i*89], signed=False, n_word=32, n_frac=0).astype(int).tolist())
        Nv.append(Fxp(val="0x"+result[26+i*89]+"0x"+result[25+i*89], signed=False, n_word=16, n_frac=0).astype(int).tolist())    
        cfo.append(Fxp(val="0x"+result[28+i*89]+"0x"+result[27+i*89], signed=True, n_word=16, n_frac=0).astype(int).tolist())
        AoA_Phase.append(Fxp(val="0x"+result[30+i*89]+"0x"+result[29+i*89], signed=False, n_word=16, n_frac=7).astype(float))   
        Fp_index.append(Fxp(val="0x"+result[32+i*89]+"0x"+result[31+i*89], signed=False, n_word=16, n_frac=6).astype(float))  
        Mp_index.append(Fxp(val="0x"+result[34+i*89]+"0x"+result[33+i*89], signed=False, n_word=16, n_frac=6).astype(float))  
        Mapping.append(Fxp(val="0x"+result[35+i*89], signed = False, n_word=8, n_frac = 0).astype(int).tolist())
        
    DNN_data += SNR_mp + SNR_fp + SNR_Total + rssi + CIR_mp + CIR_fpp + Nv

    return DNN_data