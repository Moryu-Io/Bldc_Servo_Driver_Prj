import serial
import serial.tools.list_ports
import logging


class BldcDebugIf:
    def __init__(self) -> None:
        self.C_USB_DESC = "STMicroelectronics STLink Virtual COM Port"
        self.ser = serial.Serial()
        self.ser.baudrate = 115200

        # ログ設定
        self.logger = logging.getLogger('BldcSerial')
        self.logger.setLevel(logging.DEBUG)  # ログレベル

        formatter = logging.Formatter('[%(name)s][%(levelname)s]:%(message)s')
        self.ch = logging.StreamHandler()
        self.ch.setLevel(logging.DEBUG)
        self.ch.setFormatter(formatter)
        self.logger.addHandler(self.ch)

    def __enter__(self):
        self.open()
        return self
    
    def __exit__(self, exception_type, exception_value, traceback):
        self.close()

    def open(self):
        devices = serial.tools.list_ports.comports()
        for device in devices:
            if self.C_USB_DESC in device.description:
                self.logger.debug(f'Connecting {device.description}')
                self.ser.port = device.name
                break
        
        try:
            self.ser.open()
            self.logger.debug('Success!')
        except:
            self.logger.debug('Failure...')
    
    def close(self):
        if self.ser.is_open:
            self.ser.close()
            self.logger.debug('Serial Close')

    def readline(self):
        rxstr = str(self.ser.readline().decode(encoding='utf-8').strip().replace("\x00",""))
        self.logger.debug('<read_data>:'+rxstr)
        return rxstr

    def print_ver(self):
        self.ser.write(b'b')
        print('**********')
        print(self.readline())
        print(self.readline())
        print(self.readline())
        print('**********')
            

def test():
    with BldcDebugIf() as bldc_if:
        bldc_if.print_ver()

if __name__ == '__main__':
    test()
