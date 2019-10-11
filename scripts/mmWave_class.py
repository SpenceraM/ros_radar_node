#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import os
import time
import sys
import socket
import serial
import pdb
class mmWave_Sensor():
    #  iwr1443boost configuration commands
    iwr_cfg_cmd = [ \
        'flushCfg', \
        'dfeDataOutputMode 1', \
        'channelCfg 15 1 0', \
        'adcCfg 2 1', \
        'lowPower 0 1', \
        'profileCfg 0 77 20 5 80 0 0 40 1 256 7000 0 0 30', \
        'chirpCfg 0 0 0 0 0 0 0 1', \
        'frameCfg 0 0 128 0 20 1 0', \
        'testFmkCfg 0 0 0 1', \
        'setProfileCfg disable ADC disable'
    ]
    iwr_rec_cmd = ['sensorStop', 'sensorStart']
    # dca1000evm configuration commands; only the ones used are filled in
    dca_cmd = { \
        'RESET_FPGA_CMD_CODE'               : b"", \
        'RESET_AR_DEV_CMD_CODE'             : b"", \
        'CONFIG_FPGA_GEN_CMD_CODE'          : b"\x5a\xa5\x03\x00\x06\x00\x01\x01\x01\x02\x03\x1e\xaa\xee", \
        'CONFIG_EEPROM_CMD_CODE'            : b"", \
        'RECORD_START_CMD_CODE'             : b"\x5a\xa5\x05\x00\x00\x00\xaa\xee", \
        'RECORD_STOP_CMD_CODE'              : b"\x5a\xa5\x06\x00\x00\x00\xaa\xee", \
        'PLAYBACK_START_CMD_CODE'           : b"", \
        'PLAYBACK_STOP_CMD_CODE'            : b"", \
        'SYSTEM_CONNECT_CMD_CODE'           : b"\x5a\xa5\x09\x00\x00\x00\xaa\xee", \
        'SYSTEM_ERROR_CMD_CODE'             : b"\x5a\xa5\x0a\x00\x01\x00\xaa\xee", \
        'CONFIG_PACKET_DATA_CMD_CODE'       : b"\x5a\xa5\x0b\x00\x06\x00\xc0\x05\x35\x0c\x00\x00\xaa\xee", \
        'CONFIG_DATA_MODE_AR_DEV_CMD_CODE'  : b"", \
        'INIT_FPGA_PLAYBACK_CMD_CODE'       : b"", \
        'READ_FPGA_VERSION_CMD_CODE'        : b"\x5a\xa5\x0e\x00\x00\x00\xaa\xee", \
    }

    dca_cmd_addr = ('192.168.33.180', 4096)
    dca_socket = None
    data_socket = None
    iwr_serial = None

    dca_socket_open = False
    data_socket_open = False
    serial_open = False

    capture_started = 0

    data_file = None
    data_filename = None

    def __init__(self):
        #super().__init__()

        self.dca_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dca_socket.bind(("192.168.33.30", 4096))
        self.dca_socket.settimeout(10)
        self.dca_socket_open = True

        self.data_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.data_socket.bind(("192.168.33.30",4098))
        self.data_socket.settimeout(2.5e-5)
        self.data_socket_open = True

        self.iwr_serial = serial.Serial(port='/dev/ttyACM0', baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.100)
        self.serial_open = self.iwr_serial.is_open

    def close(self):
        self.dca_socket.close()
        self.data_socket.close()
        self.iwr_serial.close()
        self.close_data_file()

    def collect_response(self):
        status = 1
        while status:
            try:
                msg, server = self.dca_socket.recvfrom(2048)
                # pdb.set_trace()
                # status = int.from_bytes(msg[4:6], byteorder='little')
                import struct
                (status,) = struct.unpack('<H', msg[4:6])

                if status == 898:
                    break
            except Exception as e:
                print(e)
                continue

    def collect_arm_response(self):
        status = 1
        while status:
            try:
                msg, server = self.dca_socket.recvfrom(2048)
                if msg == self.dca_cmd['SYSTEM_ERROR_CMD_CODE']:
                    break
            except Exception as e:
                print(e)
                continue

    def setupDCA_and_cfgIWR(self):
        if not self.dca_socket or not self.iwr_serial:
            return

        # Set up DCA
        print("SET UP DCA")
        self.dca_socket.sendto(self.dca_cmd['SYSTEM_CONNECT_CMD_CODE'], self.dca_cmd_addr)
        import sys
        print(sys.version)
        print('checkpoint1')
        self.collect_response()
        self.dca_socket.sendto(self.dca_cmd['READ_FPGA_VERSION_CMD_CODE'], self.dca_cmd_addr)
        self.collect_response()
        self.dca_socket.sendto(self.dca_cmd['CONFIG_FPGA_GEN_CMD_CODE'], self.dca_cmd_addr)
        self.collect_response()
        self.dca_socket.sendto(self.dca_cmd['CONFIG_PACKET_DATA_CMD_CODE'], self.dca_cmd_addr)
        self.collect_response()
        print("")

        # configure IWR
        print("CONFIGURE IWR")
        #Send and read a few CR to clear things in buffer. Happens sometimes during power on
        for i in range(5):
            self.iwr_serial.write('\r'.encode())
            self.iwr_serial.reset_input_buffer()
            time.sleep(.1)

        for cmd in self.iwr_cfg_cmd:
            for i in range(len(cmd)):
                self.iwr_serial.write(cmd[i].encode('utf-8'))
                time.sleep(0.010)   #  10 ms delay between characters
            self.iwr_serial.write('\r'.encode())
            self.iwr_serial.reset_input_buffer()
            time.sleep(0.010)       #  10 ms delay between characters
            time.sleep(0.100)       # 100 ms delay between lines
            response = self.iwr_serial.read(size=6)
            print('LVDS Stream:/>' + cmd)
            print(response[2:].decode())
        print("")

    def arm_dca(self):
        if not self.dca_socket:
            return

        print("ARM DCA")
        self.dca_socket.sendto(self.dca_cmd['RECORD_START_CMD_CODE'], self.dca_cmd_addr)
        self.collect_arm_response()
        print("success!")
        print("")

    def toggle_capture(self, toggle=0, id_val=0, dir_path=''):
        if not self.dca_socket or not self.iwr_serial:
            return

        # only send command if toggle != status of capture
        if toggle == self.capture_started:
            return

        sensor_cmd = self.iwr_rec_cmd[toggle]
        for i in range(len(sensor_cmd)):
            self.iwr_serial.write(sensor_cmd[i].encode('utf-8'))
            time.sleep(0.010)   #  10 ms delay between characters
        self.iwr_serial.write('\r'.encode())
        self.iwr_serial.reset_input_buffer()
        time.sleep(0.010)       #  10 ms delay between characters
        time.sleep(0.100)       # 100 ms delay between lines
        response = self.iwr_serial.read(size=6)
        print('LVDS Stream:/>' + sensor_cmd)
        print(response.decode('utf-8'))

        if sensor_cmd == 'sensorStart':
            # create path if needed
            path = dir_path
            if path != '':
                try:
                    os.stat(path)
                except:
                    os.mkdir(path)

            self.data_filename = path + str(id_val) + '_mmWaveData.bin'
            self.open_data_file()

        elif sensor_cmd == 'sensorStop':
            self.close_data_file()
            self.dca_socket.sendto(self.dca_cmd['RECORD_STOP_CMD_CODE'], self.dca_cmd_addr)
            self.collect_response()

        self.capture_started = toggle

    def open_data_file(self):
        try:
            os.remove(self.data_filename)
        except OSError:
            pass
        self.data_file = open(self.data_filename, 'wb')

    def close_data_file(self):
        if not self.data_file or not self.data_filename:
            return

        self.data_file.close()
        self.data_file = None
        self.data_filename = None

    def collect_data(self):
        if not self.data_file:
            print('No data file is opened for recording capture.')
            return

        try:
            msg, server = self.data_socket.recvfrom(2048)
            self.data_file.write(msg)
        except:
            return


