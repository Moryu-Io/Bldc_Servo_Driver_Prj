import serial
import struct
import time

COMnum = "COM7"

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


def main():
    with serial.Serial(COMnum, 115200, timeout=1) as ser:
        write_float(ser, 0x24, float(360*7/16384))
        write_byte(ser, 0x28, 1)

        print_flash(ser)

if __name__ == '__main__':
    main()