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
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QCheckBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPushButton, QVBoxLayout, QWidget
from mmWave_class import mmWave_Sensor







class mmWave_Capture_Thread(QThread):
    def run(self):
        while True:
            if mmwave_sensor.capture_started:
                mmwave_sensor.collect_data()

class App(QWidget):
    def __init__(self):
        super(App,self).__init__()
        self.title = 'Depth Capture'
        self.width = 640
        self.height = 480
        self.dir_path = ''
        self.id_val = None
        self.cur_path = 'Current Path: ' + self.dir_path + str(self.id_val) + '_file.*'

        self.mm_cap_th = mmWave_Capture_Thread(self)



        self.arm_button = QPushButton('ARM DCA')
        self.start_button = QPushButton('Start Capture')
        self.stop_button = QPushButton('Stop Capture')
        self.id_button = QPushButton('      Set ID     ')
        self.id_textbox = QLineEdit(self)
        self.dir_button = QPushButton('Set Directory')
        self.dir_textbox = QLineEdit(self)
        self.path_label = QLabel(self.cur_path)


        id_layout = QHBoxLayout()
        id_layout.addWidget(self.id_textbox)
        id_layout.addWidget(self.id_button)

        dir_layout = QHBoxLayout()
        dir_layout.addWidget(self.dir_textbox)
        dir_layout.addWidget(self.dir_button)

        self.input_box = QGroupBox('Capture Data')
        input_layout = QVBoxLayout()
        input_layout.addLayout(id_layout)
        input_layout.addLayout(dir_layout)
        input_layout.addWidget(self.path_label)
        input_layout.addWidget(self.arm_button)
        input_layout.addWidget(self.start_button)
        input_layout.addWidget(self.stop_button)
        self.input_box.setLayout(input_layout)

        main_layout = QGridLayout()
        main_layout.addWidget(self.input_box, 0, 2, 1, 1)
        self.setLayout(main_layout)

        self.initUI()


    # allows directory path to be set by the user, displays in GUI
    def set_dir(self):
        self.dir_path = self.dir_textbox.text()
        if len(self.dir_path) != 0 and self.dir_path[-1] != '/':
                self.dir_path = self.dir_path + '/'
        self.path_label.setText('Current Path: ' + self.dir_path + str(self.id_val) + '_file.*')

    # allows id to be set by the user, displays in GUI
    def set_id(self):
        self.id_val = self.id_textbox.text()
        if self.id_val == '':
            self.id_val = 0
        self.path_label.setText('Current Path: ' + self.dir_path + str(self.id_val) + '_file.*')

    # initializes the UI; sets up the threads for the sensors streams and interaction listeners for the buttons
    def initUI(self):
        # spawn window
        self.setWindowTitle(self.title)
        self.resize(1.5*self.width, 1.1*self.height)
        QApplication.processEvents()

        # # create thread for updating realsense streams
        # rs_th = RS_Thread(self)
        # rs_th.color_pixmap.connect(self.set_rs_color_data)
        # rs_th.depth_pixmap.connect(self.set_rs_depth_data)
        # rs_th.ir_L_pixmap.connect(self.set_rs_ir_L_data)
        # rs_th.ir_R_pixmap.connect(self.set_rs_ir_R_data)
        # rs_th.start()

        # create thread for the mmWave sensor

        self.mm_cap_th.start()

        # set up button listeners
        self.arm_button.clicked.connect(mmwave_sensor.arm_dca)
        self.start_button.clicked.connect(lambda: mmwave_sensor.toggle_capture(toggle=1, id_val=self.id_val, dir_path=self.dir_path))
        self.stop_button.clicked.connect(lambda: mmwave_sensor.toggle_capture(toggle=0, id_val=self.id_val, dir_path=self.dir_path))
        self.id_button.clicked.connect(self.set_id)
        self.dir_button.clicked.connect(self.set_dir)

        # show app
        self.show()


if __name__ == '__main__':

    '''try:
        mmwave_sensor = mmWave_Sensor()
        mmwave_sensor.setupDCA_and_cfgIWR()
        mmwave_sensor.arm_dca()
        self.mm_cap_th.start()
        mmwave_sensor.toggle_capture(toggle=1, id_val='ros_0', dir_path='')
        time.sleep(5.010)
        mmwave_sensor.toggle_capture(toggle=0, id_val='ros_0', dir_path='')
        mmwave_sensor.close()
    except rospy.ROSInterruptException:
        pass
    '''
    mmwave_sensor = mmWave_Sensor()
    mmwave_sensor.setupDCA_and_cfgIWR()
    try:
        app = QApplication([])
        ex = App()
        app.exec_()

    except KeyboardInterrupt:

        mmwave_sensor.close()

        sys.exit(0)