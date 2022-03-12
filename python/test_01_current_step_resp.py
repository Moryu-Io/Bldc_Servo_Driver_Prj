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

Iq_TARGET_A = 0.2
Id_TARGET_A = 0


def start_test(ser:serial.Serial):
    Cmd_List = []
    Cmd_List.append(b't')

    IqTgtHex = struct.pack('<f', float(Iq_TARGET_A))
    IdTgtHex = struct.pack('<f', float(Id_TARGET_A))

    for i in range(4):
        Cmd_List.append(struct.pack("B", IqTgtHex[i]))

    for i in range(4):
        Cmd_List.append(struct.pack("B", IdTgtHex[i]))

    print(Cmd_List)
    for cmd in Cmd_List:
        print(cmd)
        ser.write(cmd)


def main():
    ser = serial.Serial(COMnum, 115200, timeout=1)
    start_test(ser)

    time.sleep(2)

    ser.write(b'p')


    # 結果や設定値格納用変数
    IqTarget = []
    IdTarget = []
    IqMeas = []
    IdMeas = []
    VqDrv = []
    VdDrv = []


    start_time = time.time()    # タイムアウト処理用
    insp_time = "{0:%Y_%m%d_%H%M%S}".format(datetime.datetime.now())    # タイムスタンプ用

    # シリアル通信結果処理
    while ((time.time() - start_time) <= 20.0):
        rxstr = str(ser.readline().decode(encoding='utf-8').strip().replace("\x00",""))
        #print(rxstr)

        if re.match(r"Start",rxstr):
            pass
        elif re.match(r"[-+]?\d*.\d*,[-+]?\d*.\d*,[-+]?\d*.\d*,[-+]?\d*.\d*,[-+]?\d*.\d*,[-+]?\d*.\d*",rxstr):
            # ログ整形
            rxlist = rxstr.split(',')
            IqTarget.append(float(rxlist[0]))
            IdTarget.append(float(rxlist[1]))
            IqMeas.append(float(rxlist[2]))
            IdMeas.append(float(rxlist[3]))
            VqDrv.append(float(rxlist[4]))
            VdDrv.append(float(rxlist[5]))
        elif ("End" in rxstr):
            # 検査終了
            break
        
    ser.close()


    # グラフ描画
    timelist = [0.1*x for x in range(len(IqTarget))]

    fig = plt.figure()
    ax_l = fig.add_subplot(211)
    ax_r = ax_l.twinx()
    ax_l.plot(timelist,IqTarget,label="IqTgt")
    ax_l.plot(timelist,IqMeas,label="IqMeas")
    ax_r.plot(timelist,VqDrv,label="Vq",color='r')
    l_h, l_l = ax_l.get_legend_handles_labels()
    r_h, r_l = ax_r.get_legend_handles_labels()
    ax_l.legend(l_h+r_h, l_l+r_l)
    ax_l.grid(True, 'both')
    ax_l.set_xlabel('Time[ms]')
    ax_l.set_ylabel('Current[A]')
    ax_r.set_ylabel('Voltage[V]')

    ax_l = fig.add_subplot(212)
    ax_r = ax_l.twinx()
    ax_l.plot(timelist,IdTarget,label="IdTgt")
    ax_l.plot(timelist,IdMeas,label="IdMeas")
    ax_r.plot(timelist,VdDrv,label="Vd",color='r')
    l_h, l_l = ax_l.get_legend_handles_labels()
    r_h, r_l = ax_r.get_legend_handles_labels()
    ax_l.legend(l_h+r_h, l_l+r_l)
    ax_l.grid(True, 'both')
    ax_l.set_xlabel('Time[ms]')
    ax_l.set_ylabel('Current[A]')
    ax_r.set_ylabel('Voltage[V]')

    plt.show()


if __name__ == '__main__':
    main()