import serial
import struct
import time

COMnum = "COM7"

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
            ser.write(struct.pack("B", cmd))

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
CMD_MOVE_ANGLE = CanSim_MoveAngle()

def main():
    with serial.Serial(COMnum, 115200, timeout=1) as ser:
        CMD_TORQUE_ON.transmit(ser)

        RtnSmry = CanSim_RtnSummary(ser.read(12))
        RtnSmry.print_summary()

        time.sleep(3)

        CMD_MOVE_ANGLE.target_angle_deg_Q16 = -50 << 16
        CMD_MOVE_ANGLE.move_time_ms = 500
        CMD_MOVE_ANGLE.generate_datalist()
        CMD_MOVE_ANGLE.transmit(ser)


if __name__ == '__main__':
    main()