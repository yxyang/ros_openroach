# ros_openroach
This is the ros interface for OpenRoACH that allows one to communicate and control OpenRoACH using Robot Operating System. It is based on RPC-interface supported by the MBed chip.
## How it works
While there are many ways to communicate between an onboard microprocessor and host computer, one stable way we found is through the use of Remote Procedure Calls (RPC), which maintains a high baud rate and avoids losing packets.

This is the ROS package that reads IMU data from OpenRoACH, publishes it to a ROS topic and subscribes to another topic for control signals. The code to implant into the MBed Microprocessor can be found at [here](https://os.mbed.com/users/yxyang/code/ros_openroach/).

## Set Up and Troubleshooting

1. In order to set up the MBed chip to deliver data, import the code in [here](https://os.mbed.com/users/yxyang/code/ros_openroach/) to [MBed's online compiler](https://os.mbed.com/compiler/), compile the code and download the compiled `bin` file into Mbed chip

2. In a Linux computer running ROS (recommended version if ROS indigo, though other versions might work), import this Repo into the workspace, compile it and run the following command:
```
rosrun ros_openroach zumy_ros_bridge.py
```
This starts the basic interface between OpenRoACH and ROS: it publishes the gyroscope readings in topic `imu/angular_velocity` and the accelerometer readings in topic `imu/linear_velocity`. To ensure that data is flowing normally, start by running ROS's `rqt_plot` tool by running:
```
rosrun rqt_plot rqt_plot
```
and add the corresponding topics. You should see something like following:
[PLACEHOLDER]

Two common types of errors are described as following:

* In many cases Linux doesn't allow applications access to `dev/ttyACM*` ports. Since ROS is installed without `sudo` privilege, simply `sudo`ing the above command doesn't work either. Instead, one should try to give user permit to access these ports by running the following:
```
sudo chmod 666 /dev/ttyACM0
```
See [this post](https://askubuntu.com/questions/58119/changing-permissions-on-serial-port) for more details.

* In certain cases the data stream may be interrupted and computer reads mostly empty strings from OpenRoACH. Simply resetting the MBed should fix this issue.

* Lastly, if some error says that device is busy, make sure you did not have any process accessing the MBed's flash drive.

## Keyboard teleop example
Inspired by the teleop example in ROS's turtlesim tutorial, we also enabled teleop so that you can control OpenRoACH from your keyboard.

To do this, simply use the following `roslaunch` file:
```
roslaunch ros_openroach keyboard_teleop.launch
```

Then use your arrow keys to control OpenRoACH!
## Credits
* The RPC protocol as well as base functionality is inherited from the [ros_zumy]( by ROS'https://github.com/abuchan/ros_zumy) package from Austin Buchan.

* The teleop example is inspired by ROS' [turtlesim](https://github.com/ros/ros_tutorials/tree/indigo-devel/turtlesim) package.
