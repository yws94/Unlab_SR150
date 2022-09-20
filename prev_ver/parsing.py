import csv
from math import dist
from struct import *
import binascii as bi
from sys import set_coroutine_origin_tracking_depth
from threading import *
from fxpmath import Fxp

from datetime import datetime 
name = "Parsing" + datetime.now().strftime("%Y%m%d%H%M%S")  


f = open('./1615_h_test.csv', 'r', encoding='utf-8')
rdr = csv.reader(f)
data = []
data_8bit = []

for line in rdr :
    #print(line)
    #data.append(line)
    data+=line

def Converter(datas):
    fulldata = []
    text = str(datas)
    #text = "AAAA005704B7007E04800D00EA04C4002C2D2E2E2C2D2E2E0F31A62D4832372E7AC4ACC9CEC476CA30627D00B27B26003BE3910011DA330091767200B27B26003BE3910011DA33002F015000F600540002000200020002003D59A5B1F64D76A77CD08ED093D09CD040D180D080D080D041814383F6FF0E00F6FF1300F0FF0D00ECFFFBFFFAFFFDFF10000000F1FF130039FD250096F5470029F56EFE5CF6FCFB6102B6011807200AAF01E409EA00F7006007DAFBFFFF0B00F0FFFFFFF9FFFEFFF7FFFEFFFCFF0900FEFF090043FFB2FFFBFB45FE72F92EFCCFFC63FC4E03C2FF84048602250120026500A50002022A00E1018300FCFF1300FEFF0F00FEFF17000500160012000B002400F5FF4DFEB70001F70504D3F4E0058EF468010300ADFF78091D07FC051C0BDC0094029A042CFA5C08CFFBF8FFFEFFF7FF020000000700FDFFFDFFFDFF0300060000009BFF0C0040FCCFFF1CF8A4FE40FA34FD400219FEDF05CE00AC02F3016400C600BE017FFF4202B2FFE1301775"
    distance = Fxp(val="0x"+text[12:14]+"0x"+text[10:12], signed=False, n_word=16, n_frac=0).astype(int).tolist()
    AoA_azimuth = Fxp(val="0x"+text[16:18]+"0x"+text[14:16], signed=True, n_word=16, n_frac=7).astype(float)
    PDoA_azimuth = Fxp(val="0x"+text[20:22]+"0x"+text[18:20], signed=True, n_word=16, n_frac=7).astype(float)
    AoA_elevation = Fxp(val="0x"+text[24:26]+"0x"+text[22:24], signed=True, n_word=16, n_frac=7).astype(float)
    PDoA_elevation = Fxp(val="0x"+text[28:30]+"0x"+text[26:28], signed=True, n_word=16, n_frac=7).astype(float)
    nlos = Fxp(val="0x"+text[30:32], signed = False, n_word=8, n_frac = 0).astype(int).tolist()

    fulldata += [distance, AoA_azimuth, PDoA_azimuth, AoA_elevation, PDoA_elevation, nlos]
    
    #SNR_main_path
    SNR_mp_num = 32
    SNR_mp = []
    for i in range(0, 4):
        SNR_mp.append(Fxp(val="0x"+text[SNR_mp_num+i*2:SNR_mp_num+2+2*i], signed=False, n_word=8, n_frac=0).astype(int).tolist())

    #print(SNR_mp)
    fulldata += SNR_mp

    #SNR first path
    SNR_fp_num = 40
    SNR_fp = [] 
    for i in range(0, 4):
        SNR_fp.append(Fxp(val="0x"+text[SNR_fp_num+i*2:SNR_fp_num+2+2*i], signed=False, n_word=8, n_frac=0).astype(int).tolist())
    #print(SNR_fp)
    fulldata+=SNR_fp

    #SNR_Total
    SNR_Total_num = 48
    SNR_Total = []
    for i in range(0, 4):
        SNR_Total.append(Fxp(val="0x"+text[SNR_Total_num+4*i+2:SNR_Total_num+4*i+4]+"0x"+text[SNR_Total_num+4*i:SNR_Total_num+4*i+2], signed=False, n_word=16, n_frac=8).astype(float))
    #print(SNR_Total)
    #fulldata.append(SNR_Total)
    fulldata += SNR_Total

    #RSSI
    RSSI_num = 64
    RSSI = []
    for i in range(0, 4):
        RSSI.append(Fxp(val="0x"+text[RSSI_num+4*i+2:RSSI_num+4*i+4]+"0x"+text[RSSI_num+4*i:RSSI_num+4*i+2], signed=True, n_word=16, n_frac=8).astype(float))
    #print(RSSI)
    #fulldata.append(RSSI)
    fulldata += RSSI
    
    #CIR main power
    CIR_mp_num = 80
    CIR_mp = []
    for i in range(0, 4):
        CIR_mp.append(Fxp(val="0x"+text[CIR_mp_num+8*i+6:CIR_mp_num+8*i+8]+"0x"+text[CIR_mp_num+8*i+4:CIR_mp_num+8*i+6]+"0x"+text[CIR_mp_num+8*i+2:CIR_mp_num+8*i+4]+"0x"+text[CIR_mp_num+8*i:CIR_mp_num+8*i+2], signed=False, n_word=32, n_frac=0).astype(int).tolist())
    #print(CIR_mp)
    #fulldata.append(CIR_mp)
    fulldata += CIR_mp

    #CIR first path power
    CIR_fp_power_num = 112
    CIR_fp_power = []
    for i in range(0, 4):
        CIR_fp_power.append(Fxp(val="0x"+text[CIR_fp_power_num+8*i+6:CIR_fp_power_num+8*i+8]+"0x"+text[CIR_fp_power_num+8*i+4:CIR_fp_power_num+8*i+6]+"0x"+text[CIR_fp_power_num+8*i+2:CIR_fp_power_num+8*i+4]+"0x"+text[CIR_fp_power_num+8*i:CIR_fp_power_num+8*i+2], signed=False, n_word=32, n_frac=0).astype(int).tolist())
    #print(CIR_fp_power)
    #fulldata.append(CIR_fp_power)
    fulldata += CIR_fp_power

    #Noise variance 
    Nv = 144
    Noise_variance = []
    for i in range(0, 4):
        Noise_variance.append(Fxp(val="0x"+text[Nv+4*i+2:Nv+4*i+4]+"0x"+text[Nv+4*i:Nv+4*i+2], signed=False, n_word=16, n_frac=0).astype(int).tolist())    
    #print(Noise_variance)
    #fulldata.append(Noise_variance)
    fulldata += Noise_variance

    #CFO
    CFO_num = 160
    CFO = []
    for i in range(0, 4):
        CFO.append(Fxp(val="0x"+text[CFO_num+4*i+2:CFO_num+4*i+4]+"0x"+text[CFO_num+4*i:CFO_num+4*i+2], signed=True, n_word=16, n_frac=0).astype(int).tolist())
    #print(CFO)
    #fulldata.append(CFO)
    fulldata += CFO

    #AoA Phase 
    AoA_Phase_num = 176
    AoA_Phase = []
    for i in range(0, 4):
        AoA_Phase.append(Fxp(val="0x"+text[AoA_Phase_num+4*i+2:AoA_Phase_num+4*i+4]+"0x"+text[AoA_Phase_num+4*i:AoA_Phase_num+4*i+2], signed=False, n_word=16, n_frac=7).astype(float))   
    #print(AoA_Phase)
    #fulldata.append(AoA_Phase)
    fulldata += AoA_Phase

    #First Path index
    Fp_index_num = 192
    Fp_index = []
    for i in range(0, 4):
        Fp_index.append(Fxp(val="0x"+text[Fp_index_num+4*i+2:Fp_index_num+4*i+4]+"0x"+text[Fp_index_num+4*i:Fp_index_num+4*i+2], signed=False, n_word=16, n_frac=6).astype(float))  
    #print(Fp_index)
    #fulldata.append(Fp_index)
    fulldata += Fp_index

    #Main path index
    Mp_index_num = 208
    Mp_index = []
    for i in range(0, 4):
        Mp_index.append(Fxp(val="0x"+text[Mp_index_num+4*i+2:Mp_index_num+4*i+4]+"0x"+text[Mp_index_num+4*i:Mp_index_num+4*i+2], signed=False, n_word=16, n_frac=6).astype(int).tolist())  
    #print(Mp_index)
    #fulldata.append(Mp_index)
    fulldata += Mp_index


    #Mapping
    Mapping_num = 224
    Mapping = []
    for i in range(0, 4):
        Mapping.append(Fxp(val="0x"+text[Mapping_num+i*2:Mapping_num+2+2*i], signed=False, n_word=8, n_frac=0).astype(int).tolist())    
    #print(Mapping)
    #fulldata.append(Mapping)
    fulldata += Mapping


    #CIR sample 
    CIR_sample = 232
    CIR_sample_real_imag = []
    for i in range(0, 126):
        CIR_sample_real_imag.append(Fxp(val="0x"+text[(CIR_sample+4*i)+2:(CIR_sample+4*i)+4]+"0x"+text[CIR_sample+4*i:(CIR_sample+4*i)+2], signed=False, n_word=16, n_frac=0).astype(int).tolist())
    #fulldata.append(CIR_sample_real_imag)
    fulldata += CIR_sample_real_imag

    print(fulldata)
    return fulldata

#line = Converter()

for i in range(0, len(data)):
    Converter(data[i])
