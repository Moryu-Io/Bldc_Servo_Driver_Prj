import serial
import struct
import time

COMnum = "COM9"

def print_flash(ser:serial.Serial):
    cmdlist = [b'f', b'p']
    for cmd in cmdlist:
        ser.write(cmd)

    ser.timeout = 2
    pre_time = time.time()
    while True:
        line = ser.readline()
        print(line.strip().decode('UTF-8'))
        if time.time() - pre_time > 1.5:
            break

def write_byte(ser:serial.Serial, addr, val):
    value_byte_list = struct.pack('<Hb', addr, val)

    cmdlist = [b'f', b'w']
    for cmd in cmdlist:
        ser.write(cmd)
    for cmd in value_byte_list:
        ser.write(struct.pack('B', cmd))

def write_float(ser:serial.Serial, addr, val):
    value_byte_list = struct.pack('<f', val)

    for idx, d_byte in enumerate(value_byte_list):
        cmdlist = [b'f', b'w']
        for cmd in cmdlist:
            ser.write(cmd)
        idx_byte = struct.pack('<H', addr + idx)
        cmdlist = [idx_byte[0], idx_byte[1], d_byte]
        for cmd in cmdlist:
            ser.write(struct.pack('B', cmd))

def change_to_pm2505(ser:serial.Serial):
    # 小モータ用パラメータ書き換え
    write_float(ser, 0x24, float(360*7/16384))
    write_byte(ser, 0x28, 1)

def change_to_gim4305(ser:serial.Serial):
    # GIM4305用パラメータ書き換え
    write_float(ser, 0x24, float(360*14/16384))
    write_byte(ser, 0x28, 1)

def main():
    with serial.Serial(COMnum, 115200, timeout=1) as ser:
        print(' 0:flash 表示')
        print(' 1:flash 保存')
        print(' 2:flash リセット')
        print('10:位置制御PIDパラメータ書き換え')
        print('20:CAN ID書き換え')
        print('21:PM2205用書き換え')
        print('22:GIM4305用書き換え')
        mode = int(input('>> '))

        if mode == 0:
            print_flash(ser)
        elif mode == 1:
            ser.write(b'f')
            ser.write(b's')
        elif mode == 2:
            ser.write(b'f')
            ser.write(b'r')
        elif mode == 10:
            pgain = input('Pゲインは? >> ')
            igain = input('Iゲインは? >> ')
            dgain = input('Dゲインは? >> ')
            ilim = input('Iリミットは? >> ')

            if pgain == '':
                pass
            else:
                write_float(ser, 0x90,float(pgain))

            if igain == '':
                pass
            else:
                write_float(ser, 0x94,float(igain))

            if dgain == '':
                pass
            else:
                write_float(ser, 0x98,float(dgain))

            if ilim == '':
                pass
            else:
                write_float(ser, 0x9C,float(ilim))
            
            ser.write(b'f')
            ser.write(b'd')
        elif mode == 20:
            id = input('CAN IDは? >> ')
            write_byte(ser, 0x10, int(id))
        elif mode == 21:
            change_to_pm2505(ser)
        elif mode == 22:
            change_to_gim4305(ser)


if __name__ == '__main__':
    main()