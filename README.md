# ros_radar_node
The hardware components used in this demo are the IWR1443BOOST radar and the DCA1000EVM. Familiarity with the ROS framework will help here with the running of this software package.

## File Structure
The bulk of this code is run in Python 2.7 (the standard for ROS Kinetic) with the exception of the circular buffer used in the implementation. In that case, a python script calls a C function. Therefore, all ROS communication is done completely in python.

### scripts/
- **no_Qt.py** is the main python script and will be the main script that is run from the terminal. It takes a terminal argument to specify the radar configuration to be sent to the IWR1443BOOST. This script sets up the threads to both receive and publish the radar frames. Additionally, in the beginning of the execution, the radar configuration is also published.

- **mmWave_class_noQt.py** defines the class that interfaces with the radar.

- **circular_buffer.py** implements the python portion of the circular buffer and accesses an *.so (shared object)* file for the C portion of the implementation. To communicate with C, we use the ctypes library.

- **circ_buff.c** is the C code that can either add data to the buffer or the appropriate amount of zeros to account for the dropped packets. This code will not add frames that are entirely made up of zeros.

- **circ_buff.so** is the shared object file. This is necessary for the interaction with python.

- **radar_config.py** converts the configuration parameters between a properly formatted string and a dictionary. In order to send the data to the radar, the parameters must be in a string. However, for understandability and accessibility, it makes most sense to store the parameters in a dictionary in the parameter server.

- **configCMD.json** stores the configuration parameters.

- **listener.py** prints the radar frames when run at the same time as the no_QT.py script

### msg/
- **data_frame.msg** This file specifies the content of the messages to be published. As of now it is just a 1D array of 16 bit values. In the future, using our own custom message definition like this will allow for more customization.

## ROS Topics
- **/radar_data** Each published message is one radar frame. The message is a 1D int16 array. For our current test setup with 4 antennas the frames can be reformatted for post-processing with the following code:
```
import numpy as np
data = np.array(frames) # an array of n frames
data = data.reshape(-1, 8)
data = data[:, :4] + 1j * data[:, 4:]
data = data.reshape(-1)
data = data.reshape(-1, 128, 256, 4)  # where there are 128 chirps and each chirp has 256 samples
```

- **/config_str** This will publish each line in the configuration command as a string.

## Usage
1. Put the ros_radar_node directory in your catkin workspace.
2. Make sure that when you add this package to your catkin workspace, that you compile correctly. This can be done by running the following in your terminal.
```
cd catkin_ws
catkin_make
```
3. Once properly built, to run any ROS package, you must first run:
```
roscore
```
4. Make sure the necessary python scripts are executable. Run the following in the scripts/ directory.
```
chmod +x noQt.py
chmod +x listener.py
```

4. To run the test configuration from the json, open a new terminal, navigate into the scripts/ directory, and run:

xWR1443:
```
rosrun ros_radar_node noQt.py test
```

xWR1843:
```
rosrun ros_radar_node noQt.py test_1843
```

5. To make sure the radar frames are being published without saving the messages into a rosbag, run the following in a new terminal.
```
rosrun ros_radar_node listener.py
```

### Troubleshooting
One bug that may occur on different devices is using the wrong port. In `mmWave_class_noQt.py`, find the function `setupDCA_and_cfgIWR`.
Currently, the port is set to `'/dev/ttyACM0'`. This may change based on the device. An easy way to check which port to use is checking the contents of the `\dev\` folder when the devices are properly plugged in. There might be multiple `ttyACM*` items. Change the port value in the code to one of the available port names.

#TODO:
- [ ] Document 1843 1443 differences
- [ ] Better documentation for config
- [ ] Add support for 1843 object data in serial console
