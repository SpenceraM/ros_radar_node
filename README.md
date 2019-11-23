# ros-radar-node
The hardware components used in this demo are the IWR1443BOOST radar and the DCA1000EVM. Familiarity with the ROS framework will help here with the running of this software package.

## File Structure
The bulk of this code is run in Python 2.7 (the standard for ROS Kinetic) with the exception of the circular buffer used in the implementation. In that case, a python script calls a C function. Therefore, all ROS communication is done completely in python. 

### scripts/
- **no_Qt.py** is the main python script and will be the script you run from the terminal. Here, the data collection thread is created and started. This is where the computer listens for incoming packets. Additionally, the thread that publishes the radar frames is also implemented here. These two threads work together. The data collection will listen to packets and fill a circular buffer. Once a frame is completed, that thread will put the frame into the queue. Meanwhile, the publishing thread is checking this queue. If the queue is not empty, the publisher will publish the contents to the *radar_data* topic.

- **mmWave_class_noQt.py** defines the class that interfaces with the radar. Currently this also creates a binary file that stores the binary packets from  the DCA1000EVM. To write to that file, make sure the following line is uncommented: `self.data_file.write(msg)`

- **circular_buffer.py** implements the python portion of the circular buffer and accesses an *.so (shared object)* file for the C portion of the implementation. To communicate with C, we use the ctypes library.

- **circ_buff.c** This is the C code that can either add data to the buffer or the appropriate amount of zeros to account for the dropped packets.

- **circ_buff.so** This is the shared object file. This is necessary for the interaction with python.

- **radar_config.py** This converts the configuration parameters between a properly formatted string and a dictionary. In order to send the data to the radar, the parameters must be in a string. However, for understandability and accessibility, it makes most sense to store the parameters in a dictionary in the parameter server.

### msg/
- **data_frame.msg** This file specifies the content of the messages to be published. As of now it is just an array of 16 bit values. In the future, using our own custom message definition like this will allow for more customization.

## Usage

Make sure that when you add this package to your catkin workspace, that you compile correctly. This can be done by running the following in your terminal.

```
cd catkin_ws
catkin_make
```

Once properly built, to run the software package, simply run:

```
rosrun ros-radar-node noQt.py
```

Currently, you must be in the scripts/ directory so that the python code can access the .so file.

If the code cannot find the noQt.py file, check if the python file has execution privileges. This privilege can be added by simply running `sudo chmod +x no_Qt.py`



