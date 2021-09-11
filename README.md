# can_reader

An openpilot style CAN interface using opendbc, arduino, and ROS.

This package reads CAN frames sent over serial via an Arduino, and republishes the decoded frames in ROS.
- It can also be used directly with a CAN-USB device and [socketcan](http://wiki.ros.org/socketcan_interface) to publish decoded CAN frames into ROS.

## Setup
To permanently enable read to port /dev/ttyACM0
``` bash
$ sudo usermod -a -G dialout $USER
$ sudo reboot 
```

To enable for one time:
``` bash
 $ sudo chmod 666 /dev/ttyACM0 
```

## Getting Started
- Clone this repository into your catkin workspace.
- Plug in the Arduino / USB CAN Reader (default port is /dev/ttyACM0).
- Select your vehicle config (or write your own).


```
$ catkin_make
$ source devel/setup.bash
```
 and run

``` bash
$ roslaunch can_decoder can_decoder.launch
```

Topics Published:
- can_msgs/Frame: ```/mini/can``` 
- can_decoder/CarState:  ```/mini/vehicle_state```

## Known Issues:
- The mock arduino publisher sometimes causes bad frames to be sent, which will not be decoded, and sometimes crash. Just restart it.
- Hasn't been used to send CAN data back to the car.