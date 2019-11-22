#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from mmWave.msg import data_frame
from rospy.numpy_msg import numpy_msg
import os
import time
import sys
import socket
import serial
import pdb
from mmWave_class_noQt import mmWave_Sensor
import RadarRT_lib
import Queue
import threading
import pickle
from radar_config import cfg_list_to_dict

def collect_data_thread_func(mmwave_sensor):

    while True:
        if mmwave_sensor.capture_started:
            mmwave_sensor.collect_data()
            #print('-------')



def check_and_publish_thread_func(mmwave,pub):
    while True:
        if mmwave.capture_started:
                if mmwave.data_array.queue.qsize() > 0:
                    #print mmwave.data_array.queue.get()
                    #hello_str = "hello world %s" % rospy.get_time()
                    #data = mmwave.data_array.queue.get()
                    #print data.dtype
                    pub.publish(mmwave.data_array.queue.get())



def set_dir(self):
    self.dir_path = self.dir_textbox.text()
    if len(self.dir_path) != 0 and self.dir_path[-1] != '/':
            self.dir_path = self.dir_path + '/'
    self.path_label.setText('Current Path: ' + self.dir_path + str(self.id_val) + '_file.*')


if __name__ == '__main__':
    rospy.init_node('radar_collect', anonymous=True)
    pub = rospy.Publisher('radar_data', numpy_msg(data_frame), queue_size=10)
    #  initial iwr1443boost configuration commands
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
    iwr_cfg_dict = cfg_list_to_dict(iwr_cfg_cmd)
    rospy.set_param('iwr_cfg', iwr_cfg_dict)
    mmwave_sensor = mmWave_Sensor('test4_no_qt')
    mmwave_sensor.setupDCA_and_cfgIWR()

    x = threading.Thread(target=collect_data_thread_func, args=(mmwave_sensor,))
    x.setDaemon(True)
    x.start()

    y = threading.Thread(target=check_and_publish_thread_func, args=(mmwave_sensor,pub,))
    y.setDaemon(True)
    y.start()

    mmwave_sensor.arm_dca()
    time.sleep(2)


    try:
        mmwave_sensor.toggle_capture(toggle=1)
        #time.sleep(6)
        #mmwave_sensor.toggle_capture(toggle=0)
        rate = rospy.Rate(.5)  # 1hz
        # while not rospy.is_shutdown():
        #     rate.sleep()
        rospy.spin()
        #pdb.set_trace()
        #print len(mmwave_sensor.data_array.data)
        #pdb.set_trace()
        #print mmwave_sensor.data_array.data[:250]
        #print mmwave_sensor.data_array.data[20000:20250]
        '''
        total = []
        while mmwave_sensor.data_array.queue.qsize() > 0:
            total += mmwave_sensor.data_array.queue.get().tolist()
        pickle.dump(total, open('our_queue.dat', 'wb'))
        '''
        mmwave_sensor.toggle_capture(toggle=0)

        mmwave_sensor.close()

        sys.exit(0)

    except rospy.ROSInterruptException:
        mmwave_sensor.toggle_capture(toggle=0)
        mmwave_sensor.close()
        sys.exit(0)
        '''except KeyboardInterrupt:
        mmwave_sensor.toggle_capture(toggle=0)
        mmwave_sensor.close()
        sys.exit(0)'''