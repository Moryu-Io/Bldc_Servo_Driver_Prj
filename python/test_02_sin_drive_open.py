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

Vq_TARGET_V = 2.0
Vd_TARGET_V = 0


def start_test(ser:serial.Serial):
    Cmd_List = []
    Cmd_List.append(b't')
    Cmd_List.append(b's')

    VqTgtHex = struct.pack('<f', float(Vq_TARGET_V))
    VdTgtHex = struct.pack('<f', float(Vd_TARGET_V))

    for i in range(4):
        Cmd_List.append(struct.pack("B", VqTgtHex[i]))

    for i in range(4):
        Cmd_List.append(struct.pack("B", VdTgtHex[i]))

    print(Cmd_List)
    for cmd in Cmd_List:
        print(cmd)
        ser.write(cmd)


def main():
    ser = serial.Serial(COMnum, 115200, timeout=1)
    start_test(ser)

    time.sleep(2)
    ser.write(b's')

    time.sleep(2)
    ser.write(b'p')


    # 結果や設定値格納用変数
    Iu = []
    Iv = []
    Iw = []

    start_time = time.time()    # タイムアウト処理用
    insp_time = "{0:%Y_%m%d_%H%M%S}".format(datetime.datetime.now())    # タイムスタンプ用

    # シリアル通信結果処理
    while ((time.time() - start_time) <= 20.0):
        rxstr = str(ser.readline().decode(encoding='utf-8').strip().replace("\x00",""))
        print(rxstr)

        if re.match(r"Start",rxstr):
            pass
        elif re.match(r"[-+]?\d*.\d*,[-+]?\d*.\d*,[-+]?\d*.\d*,[-+]?\d*.\d*",rxstr):
            # ログ整形
            rxlist = rxstr.split(',')
            Iu.append(float(rxlist[0]))
            Iv.append(float(rxlist[1]))
            Iw.append(float(rxlist[2]))
        elif ("End" in rxstr):
            # 検査終了
            break


    #ser.write(b'e')
        
    ser.close()


    # グラフ描画
    timelist = [0.1*x for x in range(len(Iu))]

    fig = plt.figure()
    ax_l = fig.add_subplot(111)
    ax_l.plot(timelist,Iu,label="Iu")
    ax_l.plot(timelist,Iv,label="Iv")
    ax_l.plot(timelist,Iw,label="Iw")
    l_h, l_l = ax_l.get_legend_handles_labels()
    ax_l.legend(l_h, l_l)
    ax_l.grid(True, 'both')
    ax_l.set_xlabel('Time[ms]')
    ax_l.set_ylabel('Current[A]')

    plt.show()


if __name__ == '__main__':
    main()