import os
import numpy as np
import Queue
from ctypes import *
from numpy.ctypeslib import ndpointer
import threading

class ring_buffer:
    def __init__(self, max_len, frame_size, dtype=np.int16):
        self.max_len = c_longlong(max_len)
        self.data = np.zeros(max_len, dtype=dtype)
        self.queue = Queue.Queue()
        self.put_idx = c_longlong(0)
        self.frame_size = c_longlong(frame_size)
        self.pop_array = c_int16(-1)
        self.total = []
        
        if max_len % frame_size == 0:
            self.n_frames = max_len / frame_size
        else:
            raise ValueError("Must be multiple of frame size")

        self.c_file = CDLL(os.path.join(os.getcwd(),'circ_buff.so'))
        
        self.c_file.add_zeros.argtypes = [  
                                            c_longlong,
                                            ndpointer(c_int16, flags="C_CONTIGUOUS"),
                                            c_longlong,
                                            POINTER(c_longlong),
                                            c_longlong,
                                            POINTER(c_int16)
                                         ]

        self.c_file.add_msg.argtypes =   [    
                                            ndpointer(c_int16, flags="C_CONTIGUOUS"),
                                            c_int16,
                                            ndpointer(c_int16, flags="C_CONTIGUOUS"),
                                            c_longlong,
                                            POINTER(c_longlong),
                                            c_longlong, 
                                            POINTER(c_int16)
                                         ]

    '''
    def put(self,new_array):

        N = len(new_array)
        # Do not have to worry about exceeding length of buffer and starting at 0
        if self.put_idx + N <= self.max_len:
            self.data[self.put_idx:self.put_idx+N] = new_array
            self.put_idx = self.put_idx+N

            # push to Queue if complete data frames are available
            for n in range(self.get_idx + self.frame_size, self.max_len + 1, self.frame_size):
                if self.put_idx >= n:
                    self.queue.put(list(self.data[self.get_idx:self.get_idx + self.frame_size]))
                    self.get_idx = n
                else: break
        else:
            # The new array will loop back to the beginning of buffer
            boundary_idx = self.max_len - self.put_idx
            self.data[self.put_idx:] = new_array[:boundary_idx]

            # push frames to Queue before looping back
            for n in range(self.get_idx + self.frame_size, self.max_len+1, self.frame_size):
                # need to push to queue all remaining until end of buffer
                self.queue.put(list(self.data[self.get_idx:self.get_idx + self.frame_size]))
                self.get_idx = n

            # Determine how many more elements need to be added to beginning of buffer
            self.data[:N - boundary_idx] = new_array[boundary_idx:]
            self.put_idx = N - boundary_idx
            self.get_idx = 0
            # Push frames beginning at the start of the buffer
            for n in range(self.get_idx + self.frame_size, self.max_len + 1, self.frame_size):
                if self.put_idx >= n:
                    self.queue.put(list(self.data[self.get_idx:self.get_idx + self.frame_size]))
                    self.get_idx = n
                else:break

        # If the new array finishes at the end of the buffer
        if self.put_idx == self.max_len:
            self.put_idx = 0
            self.get_idx = 0
    '''

    def add_zeros(self,num_zeros):
        self.c_file.add_zeros(num_zeros,
                              self.data,
                              self.max_len,
                              byref(self.put_idx),
                              self.frame_size,
                              byref(self.pop_array))
        self.add_to_queue()
        #print '[P] Current put:', self.put_idx.value, '| Pop index:', self.pop_array.value


    def add_msg(self, msg):

        self.c_file.add_msg(msg,
                            len(msg),
                            self.data,
                            self.max_len,
                            byref(self.put_idx),
                            self.frame_size,
                            byref(self.pop_array))
        self.add_to_queue()
        #print '[P] Current put:', self.put_idx.value, '| Pop index:', self.pop_array.value



    def add_to_queue(self):
        #print '[P] Current put:', self.put_idx.value, '| Pop index:', self.pop_array.value
        if self.pop_array.value != -1:
            #print '-.' * 20
            data = self.data[self.frame_size.value * self.pop_array.value:self.frame_size.value * (self.pop_array.value + 1)].copy()
            #print 'Assembled', data
            #print 'pop array ',self.pop_array.value, 'frame size c ', self.frame_size.value
            self.queue.put(data)

def check_and_publish(buffer):
    global count_for_quitting
    while count_for_quitting < 35:
            if buffer.queue.qsize() > 0:
                print buffer.queue.get()
                count_for_quitting += 1


if __name__ == "__main__":
    FRAME_SIZE = 11
    PACKET_SIZE = 3
    NMSGS = 200
    buffer = ring_buffer(int(3*FRAME_SIZE), int(FRAME_SIZE))
    count_for_quitting = 0

    x = threading.Thread(target=check_and_publish, args=(buffer,))
    #x.setDaemon(True)
    x.start()


    msgs = [
        {
            'seqn': i,
            'data': i*np.ones(PACKET_SIZE,dtype=np.int16) #np.random.randint(1000, size=(PACKET_SIZE), dtype=np.uint16)
        }
        for i in range(NMSGS)
    ]

    msgs = msgs[:2] + msgs[15:]

    seqn = -1
    for msg in msgs:
        new_seqn = msg['seqn']
        data = msg['data']

        if seqn + 1 != new_seqn:
            print("Missing seq {}-{}: ".format(seqn+1, new_seqn-1))
            num_zeros = c_longlong((new_seqn - seqn - 1) * PACKET_SIZE)
            print 'num zeros ', num_zeros.value
            buffer.add_zeros(num_zeros)
            print '-'*20

        #print 'Delivered data', data
        buffer.add_msg(data)

        seqn = new_seqn
        #print('='*30)

    while buffer.queue.qsize()>0:
         print buffer.queue.get()
    x.join()
    print 'Final result', buffer.total