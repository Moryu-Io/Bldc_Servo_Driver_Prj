import serial
import struct
import re
import matplotlib.pyplot as plt
import time
import datetime
import csv
import numpy as np


SaveFolderPath = "./LOG"
COMnum = "COM7"

POS_TARGET_DEG = -90
MOVE_TIME_MS = 200
MABIKI = 4


def start_test(ser:serial.Serial):
    Cmd_List = []
    Cmd_List.append(b't')
    Cmd_List.append(b'p')

    TgtHex = struct.pack('<h', (POS_TARGET_DEG))
    MtmHex = struct.pack('<H', (MOVE_TIME_MS))
    MbkHex = struct.pack('B', (MABIKI))

    for i in range(2):
        Cmd_List.append(struct.pack("B", TgtHex[i]))

    for i in range(2):
        Cmd_List.append(struct.pack("B", MtmHex[i]))
    
    Cmd_List.append(MbkHex)

    print(Cmd_List)
    for cmd in Cmd_List:
        #print(cmd)
        ser.write(cmd)


def main():
    ser = serial.Serial(COMnum, 115200, timeout=1)
    ser.write(b'd')
    time.sleep(1.5)

    start_test(ser)

    time.sleep(2)

    ser.write(b'p')


    # 結果や設定値格納用変数
    TgtDeg = []
    PosDeg = []
    IqTarget = []
    IqMeas = []

    start_time = time.time()    # タイムアウト処理用
    insp_time = "{0:%Y_%m%d_%H%M%S}".format(datetime.datetime.now())    # タイムスタンプ用

    # シリアル通信結果処理
    while ((time.time() - start_time) <= 20.0):
        rxstr = str(ser.readline().decode(encoding='utf-8').strip().replace("\x00",""))
        #print(rxstr)

        if re.match(r"Start",rxstr):
            pass
        elif re.match(r"[-+]?\d*.\d*,[-+]?\d*.\d*,[-+]?\d*.\d*,[-+]?\d*.\d*",rxstr):
            # ログ整形
            rxlist = rxstr.split(',')
            TgtDeg.append(float(rxlist[0]))
            PosDeg.append(float(rxlist[1]))
            IqTarget.append(float(rxlist[2]))
            IqMeas.append(float(rxlist[3]))
        elif ("End" in rxstr):
            # 検査終了
            break
        
    ser.close()

    # グラフ描画
    timelist = [0.1*x*(MABIKI+1) for x in range(len(TgtDeg))]

    fig = plt.figure()
    ax_l = fig.add_subplot(211)
    ax_l.plot(timelist,TgtDeg,label="TargetPos")
    ax_l.plot(timelist,PosDeg,label="MeasPos")
    l_h, l_l = ax_l.get_legend_handles_labels()
    ax_l.legend(l_h, l_l)
    ax_l.grid(True, 'both')
    ax_l.set_xlabel('Time[ms]')
    ax_l.set_ylabel('Angle[deg]')

    ax_l = fig.add_subplot(212)
    ax_l.plot(timelist,IqTarget,label="IqTgt")
    ax_l.plot(timelist,IqMeas,label="IqMeas")
    l_h, l_l = ax_l.get_legend_handles_labels()
    ax_l.legend(l_h, l_l)
    ax_l.grid(True, 'both')
    ax_l.set_xlabel('Time[ms]')
    ax_l.set_ylabel('Current[A]')

    plt.show()


if __name__ == '__main__':
    main()