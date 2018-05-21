# ros_openroach
This is the ROS interface for OpenRoACH that allows one to communicate and control OpenRoACH using Robot Operating System installed on Raspberry Pi. It is based on RPC-interface supported by the MBed chip.

The 'src' directory also contains the code used for gait-controlled walking described [here](https://drive.google.com/file/d/1BtqgeBJKf_vY9w3NDAzwl1sBqKWttHX7/view?usp=sharing).
## How the interface works
While there are many ways to communicate between an onboard microprocessor and host computer, one stable way we found is through the use of Remote Procedure Calls (RPC), which maintains a high baud rate and avoids losing packets.

This is the ROS package that reads IMU data from OpenRoACH, publishes it to a ROS topic and subscribes to another topic for control signals. The code to implant into the MBed Microprocessor can be found at [here](https://os.mbed.com/users/yxyang/code/ros_openroach/).

## Basic Set Up and Troubleshooting
1. Powering OpenRoACH and Raspberry Pi
* Powering OpenRoACH
The Zumy circuit board on OpenRoACH takes 7.4V DC, which can be supplied either from a standard 2-cell LiPo battery or a DC power supply. Note that if the LiPo battery is also powering RaspberryPi, the battery life would not last very long --- around 30 minutes.
* Powering Raspberry Pi
There are two ways to power the Raspberry Pi: using its own power supply, or using GPIO pins. For power supply, just plug in the provided power supply to the micro-usb port. OpenRoACH and RaspberryPi will be common-grounded through the mini-usb cable. You can also power Raspberry Pi using its GPIO pins (checkout layout [here](https://docs.microsoft.com/en-us/windows/iot-core/learn-about-hardware/pinmappings/pinmappingsrpi)) by connecting the GND and 5V pins to the corresponding pins on the Zumy board. Note that when the motor starts moving it might leads to a high current that the power supply cannot take, which could in turn trigger a reset of the RaspberryPi chip.

2. Connection and wiring
Two connections need to be made between RaspberryPi and OpenRoACH. First, connect the MiniUsb on Mbed to any of the USB port on RaspberryPi (preferably /dev/ttyACM0). Then, please connect __two extra jumper wire__ between the two wheel encoders (see Zumy circuit board layout for more information) and __pin23 (left) and pin24 (right)__. This is because the serial RPC interface is blocking and does not coordinate well with the high interrupt rates of the encoder.

3. RaspberryPi and ROS setup
First, install Ubuntu Mate (16.04) onto RaspberryPi following the instructions [here](http://ubuntu-mate.org/download/)
Then, install ROS Kinect to RaspberryPi following the instructions [here](http://wiki.ros.org/kinetic/Installation)
UbuntuMate on RaspberryPi should come with `pigpio` installed. Try `import pigpio` on a python session. If it doesn't work, follow the link [here](http://abyz.me.uk/rpi/pigpio/download.html).


4. Mbed embedded processor setup
In order to set up the MBed chip to deliver data, import the code in [here](https://os.mbed.com/users/yxyang/code/ros_openroach/) to [MBed's online compiler](https://os.mbed.com/compiler/), compile the code and download the compiled `bin` file into Mbed chip

5. Have fun!
In a Linux computer running ROS (recommended version if ROS indigo, though other versions might work), import this Repo into the workspace, compile it and run the following command:
```
rosrun ros_openroach zumy_ros_bridge.py
```
This starts the basic interface between OpenRoACH and ROS: it publishes the gyroscope readings in topic `imu/angular_velocity` and the accelerometer readings in topic `imu/linear_velocity`. To ensure that data is flowing normally, start by running ROS's `rqt_plot` tool by running:
```
rosrun rqt_plot rqt_plot
```
and add the corresponding topics. You should see something like following:
![](https://github.com/yxyang/ros_openroach/blob/master/imgs/demo.png)


Two common types of errors are described as following:

* In many cases Linux doesn't allow applications access to `dev/ttyACM*` ports. Since ROS is installed without `sudo` privilege, simply `sudo`ing the above command doesn't work either. Instead, one should try to give user permit to access these ports by running the following:
```
sudo chmod 666 /dev/ttyACM0
```
See [this post](https://askubuntu.com/questions/58119/changing-permissions-on-serial-port) for more details.

* In certain cases the data stream may be interrupted and computer reads mostly empty strings from OpenRoACH. Simply resetting the MBed should fix this issue.

* Lastly, if some error says that device is busy, make sure you did not have any process accessing the MBed's flash drive.

# Examples of use
## Keyboard teleop
Inspired by the teleop example in ROS's turtlesim tutorial, we also enabled teleop so that you can control OpenRoACH from your keyboard.

To do this, simply use the following `roslaunch` file:
```
roslaunch ros_openroach keyboard_teleop.launch
```

Then use your arrow keys to control OpenRoACH!

## Gait-controlled walking
## Closing the loop with OptiTrack
# Credits
* The RPC protocol as well as base functionality is inherited from the [ros_zumy](https://github.com/abuchan/ros_zumy) package from Austin Buchan.

* The teleop example is inspired by ROS' [turtlesim](https://github.com/ros/ros_tutorials/tree/indigo-devel/turtlesim) package.
