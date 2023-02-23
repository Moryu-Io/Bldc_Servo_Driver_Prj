import serial
import struct
import time

COMnum = "COM12"

DEBUG_PRINT_ON = True

class CanSim_TxCmdBase:
    def __init__(self, struct_format):
        self.CmdStruct = struct.Struct(struct_format)
        self.CmdId = 0
        self.Dlc = 8
        self.DataList = []
    
    def transmit(self, ser:serial.Serial):
        ser.write(b'c')  # CAN模擬コマンドフラグ送信
        cmdlist = self.CmdStruct.pack(self.CmdId, self.Dlc, *self.DataList)
        for cmd in cmdlist:
            put_data = struct.pack("B", cmd)
            if DEBUG_PRINT_ON:
                print(put_data)
            ser.write(put_data)

class CanSim_TorqueOn(CanSim_TxCmdBase):
    def __init__(self):
        super().__init__('<L9B')
        self.CmdId = 0x8001
        self.Dlc = 8
        self.DataList = [0] * 8

class CanSim_TorqueOff(CanSim_TxCmdBase):
    def __init__(self):
        self.CmdStruct = struct.Struct('<L9B')
        self.CmdId = 0x8002
        self.Dlc = 8
        self.DataList = [0] * 8

class CanSim_TorqueCtrlMode(CanSim_TxCmdBase):
    def __init__(self):
        self.CmdStruct = struct.Struct('<L9B')
        self.CmdId = 0x8003
        self.Dlc = 8
        self.DataList = [0] * 8

class CanSim_MoveAngle(CanSim_TxCmdBase):
    def __init__(self):
        self.CmdStruct = struct.Struct('<LBlHH')
        self.CmdId = 0x8010
        self.Dlc = 8
        self.DataList = [0] * 3
        self.target_angle_deg_Q16 = 0
        self.move_time_ms = 0
        self.current_lim_A_Q8 = 0

    def generate_datalist(self):
        self.DataList[0] = self.target_angle_deg_Q16
        self.DataList[1] = self.move_time_ms
        self.DataList[2] = self.current_lim_A_Q8

class CanSim_AngleInit(CanSim_TxCmdBase):
    def __init__(self):
        self.CmdStruct = struct.Struct('<L5Bl')
        self.CmdId = 0x8011
        self.Dlc = 8
        self.DataList = [0] * 5

        self.set_angle_flag = 0
        self.init_angle_deg_Q16 = 0

    def generate_datalist(self):
        self.DataList[0] = self.set_angle_flag

        self.DataList[4] = self.init_angle_deg_Q16

class CanSim_SetTargetCurrent(CanSim_TxCmdBase):
    def __init__(self):
        self.CmdStruct = struct.Struct('<LBll')
        self.CmdId = 0x8110
        self.Dlc = 8
        self.DataList = [0] * 2
        self.target_iq_A_Q16 = 0
        self.target_id_A_Q16 = 0

    def generate_datalist(self):
        self.DataList[0] = self.target_iq_A_Q16
        self.DataList[1] = self.target_id_A_Q16

class CanSim_RtnSummary:
    def __init__(self, RtnBin):
        self.SummarySt = struct.unpack('<LBBhbbBb', RtnBin)
        self.CmdId = self.SummarySt[0]
        self.fault_status = self.SummarySt[1]
        self.now_mode = self.SummarySt[2]
        self.out_and_deg_Q4 = self.SummarySt[3]
        self.motor_curr_A_Q4 = self.SummarySt[4]
        self.motor_vol_V_Q3 = self.SummarySt[5]
        self.vm_V_Q3 = self.SummarySt[6]
        self.motor_tempr_deg = self.SummarySt[7]

    def print_summary(self):
        print(f'FaultStatus     : {self.fault_status:#010b}')
        print(f'now_mode        : {self.now_mode:#04x}')
        print(f'out_angle_deg   : {self.out_and_deg_Q4/16}')
        print(f'motor_curr_A    : {self.motor_curr_A_Q4/16}')
        print(f'motor_vol_V     : {self.motor_vol_V_Q3/8}')
        print(f'vm_V            : {self.vm_V_Q3/8}')
        print(f'motor_tempr_deg : {self.motor_tempr_deg}')


CMD_TORQUE_ON = CanSim_TorqueOn()
CMD_TORQUE_OFF = CanSim_TorqueOff()
CMD_TORQUE_CTRL = CanSim_TorqueCtrlMode()
CMD_MOVE_ANGLE = CanSim_MoveAngle()
CMD_ANGLE_INIT = CanSim_AngleInit()
CMD_SET_CURRENT = CanSim_SetTargetCurrent()

def main():
    with serial.Serial(COMnum, 115200, timeout=1) as ser:
        print(' 0:torque on')
        print(' 1:torque off')
        print(' 2:torque control mode')
        print('10:角度移動')
        print('11:角度初期化(現在位置)')
        print('12:角度初期化(指定位置)')
        print('100:電流指示')
        mode = int(input('>> '))

        if mode == 0:
            CMD_TORQUE_ON.transmit(ser)
        elif mode == 1:
            CMD_TORQUE_OFF.transmit(ser)
        elif mode == 2:
            CMD_TORQUE_CTRL.transmit(ser)
        elif mode == 10:
            tgt_angle = input('指示角度[deg]は？ >> ')
            move_ms = input('移動時間[ms]は？ >> ')
            CMD_MOVE_ANGLE.target_angle_deg_Q16 = int(tgt_angle) << 16
            CMD_MOVE_ANGLE.move_time_ms = int(move_ms)
            CMD_MOVE_ANGLE.generate_datalist()
            CMD_MOVE_ANGLE.transmit(ser)
            time.sleep(int(move_ms)/1000 + 1)
        elif mode == 11:
            CMD_ANGLE_INIT.set_angle_flag = 0
            CMD_ANGLE_INIT.generate_datalist()
            CMD_ANGLE_INIT.transmit(ser)
        elif mode == 12:
            tgt_angle = input('指定角度[deg]は？ >> ')
            CMD_ANGLE_INIT.set_angle_flag = int(1)
            CMD_ANGLE_INIT.init_angle_deg_Q16 = int(tgt_angle) << 16
            CMD_ANGLE_INIT.generate_datalist()
            CMD_ANGLE_INIT.transmit(ser)
        elif mode == 100:
            tgt_iq_A = float(input('Iq[A]は？ >> '))
            tgt_id_A = float(input('Id[A]は？ >> '))
            CMD_SET_CURRENT.target_iq_A_Q16 = int(tgt_iq_A * 65536)
            CMD_SET_CURRENT.target_id_A_Q16 = int(tgt_id_A * 65536)
            print(CMD_SET_CURRENT.target_iq_A_Q16 )
            CMD_SET_CURRENT.generate_datalist()
            CMD_SET_CURRENT.transmit(ser)
        else:
            CMD_TORQUE_OFF.transmit(ser)

        RtnSmry = CanSim_RtnSummary(ser.read(12))
        RtnSmry.print_summary()

        time.sleep(3)


if __name__ == '__main__':
    main()