# 外部ライブラリ
import struct
import time

# ローカルライブラリ
from test_mod_interface import BldcDebugIf


def print_flash(bldcif:BldcDebugIf):
    cmdlist = [b'f', b'p']
    for cmd in cmdlist:
        bldcif.ser.write(cmd)

    bldcif.ser.timeout = 2
    pre_time = time.time()
    while True:
        line = bldcif.ser.readline()
        print(line.strip().decode('UTF-8'))
        if time.time() - pre_time > 1.5:
            break

def write_byte(bldcif:BldcDebugIf, addr, val):
    value_byte_list = struct.pack('<Hb', addr, val)

    cmdlist = [b'f', b'w']
    for cmd in cmdlist:
        bldcif.ser.write(cmd)
    for cmd in value_byte_list:
        bldcif.ser.write(struct.pack('B', cmd))

def write_float(bldcif:BldcDebugIf, addr, val):
    value_byte_list = struct.pack('<f', val)

    for idx, d_byte in enumerate(value_byte_list):
        cmdlist = [b'f', b'w']
        for cmd in cmdlist:
            bldcif.ser.write(cmd)
        idx_byte = struct.pack('<H', addr + idx)
        cmdlist = [idx_byte[0], idx_byte[1], d_byte]
        for cmd in cmdlist:
            bldcif.ser.write(struct.pack('B', cmd))

def change_to_pm2505(bldcif:BldcDebugIf):
    # 小モータ用パラメータ書き換え
    write_float(bldcif.ser, 0x24, float(360*7/16384))
    write_byte(bldcif.ser, 0x28, 1)

def change_to_gim4305(bldcif:BldcDebugIf):
    # GIM4305用パラメータ書き換え
    write_float(bldcif.ser, 0x24, float(-360*14/16384))
    write_byte(bldcif.ser, 0x28, -1)

def main():
    with BldcDebugIf() as bldc_if:
        print(' 0:flash 表示')
        print(' 1:flash 保存')
        print(' 2:flash リセット')
        print('10:位置制御PIDパラメータ書き換え')
        print('20:CAN ID書き換え')
        print('21:PM2205用書き換え')
        print('22:GIM4305用書き換え')
        mode = int(input('>> '))

        if mode == 0:
            print_flash(bldc_if)
        elif mode == 1:
            bldc_if.ser.write(b'f')
            bldc_if.ser.write(b's')
        elif mode == 2:
            bldc_if.ser.write(b'f')
            bldc_if.ser.write(b'r')
        elif mode == 10:
            pgain = input('Pゲインは? >> ')
            igain = input('Iゲインは? >> ')
            dgain = input('Dゲインは? >> ')
            ilim = input('Iリミットは? >> ')

            if pgain == '':
                pass
            else:
                write_float(bldc_if, 0x90,float(pgain))

            if igain == '':
                pass
            else:
                write_float(bldc_if, 0x94,float(igain))

            if dgain == '':
                pass
            else:
                write_float(bldc_if, 0x98,float(dgain))

            if ilim == '':
                pass
            else:
                write_float(bldc_if, 0x9C,float(ilim))
            
            bldc_if.ser.write(b'f')
            bldc_if.ser.write(b'd')
        elif mode == 20:
            id = input('CAN IDは? >> ')
            write_byte(bldc_if, 0x10, int(id))
        elif mode == 21:
            change_to_pm2505(bldc_if)
        elif mode == 22:
            change_to_gim4305(bldc_if)


if __name__ == '__main__':
    main()