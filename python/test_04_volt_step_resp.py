# 外部ライブラリ
import struct
import re
import matplotlib.pyplot as plt
import time
import datetime
import csv
import numpy as np

# ローカルライブラリ
from test_mod_interface import BldcDebugIf


SaveFolderPath = "./LOG"

Vq_TARGET_A = 2
Vd_TARGET_A = 0


def start_test(bldcif:BldcDebugIf):
    Cmd_List = []
    Cmd_List.append(b't')
    Cmd_List.append(b'v')

    VqTgtHex = struct.pack('<f', float(Vq_TARGET_A))
    VdTgtHex = struct.pack('<f', float(Vd_TARGET_A))

    for i in range(4):
        Cmd_List.append(struct.pack("B", VqTgtHex[i]))

    for i in range(4):
        Cmd_List.append(struct.pack("B", VdTgtHex[i]))

    print(Cmd_List)
    for cmd in Cmd_List:
        print(cmd)
        bldcif.ser.write(cmd)


def main():
    with BldcDebugIf() as bldc_if:
        start_test(bldc_if)
        time.sleep(2)
        bldc_if.ser.write(b'p')

        # 結果や設定値格納用変数
        OutAngle = []
        IqMeas = []
        IdMeas = []
        VqDrv = []
        VdDrv = []

        start_time = time.time()    # タイムアウト処理用
        insp_time = "{0:%Y_%m%d_%H%M%S}".format(datetime.datetime.now())    # タイムスタンプ用

        # シリアル通信結果処理
        while ((time.time() - start_time) <= 20.0):
            rxstr = bldc_if.readline()

            if re.match(r"Start",rxstr):
                pass
            elif re.match(r"[-+]?\d*.\d*,[-+]?\d*.\d*,[-+]?\d*.\d*,[-+]?\d*.\d*,[-+]?\d*.\d*",rxstr):
                # ログ整形
                rxlist = rxstr.split(',')
                OutAngle.append(float(rxlist[0]))
                IqMeas.append(float(rxlist[1]))
                IdMeas.append(float(rxlist[2]))
                VqDrv.append(float(rxlist[3]))
                VdDrv.append(float(rxlist[4]))
            elif ("End" in rxstr):
                # 検査終了
                break

    # グラフ描画
    timelist = [0.05*x for x in range(len(OutAngle))]

    fig = plt.figure()
    ax_ang = fig.add_subplot(311)
    ax_ang.plot(timelist,OutAngle,label="Angle[deg]")
    l_h, l_l = ax_ang.get_legend_handles_labels()
    ax_ang.legend(l_h, l_l)
    ax_ang.grid(True, 'both')
    ax_ang.set_xlabel('Time[ms]')
    ax_ang.set_ylabel('Angle[deg]')

    ax_q = fig.add_subplot(312, sharex=ax_ang)
    ax_q_r = ax_q.twinx()
    ax_q.plot(timelist,VqDrv,label="Vq")
    ax_q_r.plot(timelist,IqMeas,label="IqMeas",color='r')
    l_h, l_l = ax_q.get_legend_handles_labels()
    r_h, r_l = ax_q_r.get_legend_handles_labels()
    ax_q.legend(l_h+r_h, l_l+r_l)
    ax_q.grid(True, 'both')
    ax_q.set_xlabel('Time[ms]')
    ax_q.set_ylabel('Voltage[V]')
    ax_q_r.set_ylabel('Current[A]')

    ax_d = fig.add_subplot(313, sharex=ax_ang)
    ax_d_r = ax_d.twinx()
    ax_d.plot(timelist,VdDrv,label="Vd")
    ax_d_r.plot(timelist,IdMeas,label="IdMeas",color='r')
    l_h, l_l = ax_d.get_legend_handles_labels()
    r_h, r_l = ax_d_r.get_legend_handles_labels()
    ax_d.legend(l_h+r_h, l_l+r_l)
    ax_d.grid(True, 'both')
    ax_d.set_xlabel('Time[ms]')
    ax_d.set_ylabel('Voltage[V]')
    ax_d_r.set_ylabel('Current[A]')

    
    logfile = f"{SaveFolderPath}/VoltStep_{insp_time}.csv"
    pngfile = f"{SaveFolderPath}/VoltStep_{insp_time}.png"

    plt.savefig(pngfile)
    plt.show()
    with open(logfile, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Time[ms]", "Angle[deg]", "Vq[V]", "Vd[V]", "Iq[A]", "Id[A]"])
        for el  in list(zip(timelist, OutAngle, VqDrv, VdDrv, IqMeas, IdMeas)):
            writer.writerow(el)





if __name__ == '__main__':
    main()