# fanuc_ros_control
control fanuc through ros

## dependencies
1. build the packages [ros-industrial/fanuc](https://github.com/ros-industrial/fanuc)

```
# change to the root of the Catkin workspace
$ cd $HOME/catkin_ws

# retrieve the latest development version of fanuc.
# NOTE: 'melodic-devel' is compatible with ROS Noetic. 'kinetic' may not be
$ git clone -b melodic-devel https://github.com/ros-industrial/fanuc.git src/fanuc

# depending on the software installed on the machine
$ rosdep update

# be sure to change 'melodic' to whichever ROS release you are using
$ rosdep install --from-paths src/ --ignore-src --rosdistro melodic

# build the workspace (using catkin_tools)
$ catkin build
```

2. activate the workspace to get access to the packages just built:

```
$ source $HOME/catkin_ws/devel/setup.bash
```

3. pandas
```
pip install pandas
```

## start ROS server on fanuc controller
Assuming the fanuc ROS driver has been installed on the fanuc controller.

1. Restart the controller
  - Switch the key to "ON" on pendant, enable the pendant control
  - Press "Func", on page 3, select "1. Abort All" twice until the green button light off
  - Press "Func", on page 2, select "8. Power Cycle", restart the fanuc controller

Comments: when there is an error occurs using ROS driving the fanuc CR35ia, it is not always necessary to retart the controller, sometime, you just need to abort all running program and restart the program.

2. Confirm payload
  - Once the controller is restarted, press "confirm", enter code "4077" to confirm
  - Press "Yes", util the "STOP" change to "SAFT"

3. Start ROS server on controller
  - On pendant, press "Select" to choose "ROS" to start, and switch the key to "OFF"
  - On controller box, switch the key to "AUTO", press "Reset" button (blue) and then press "Start" button (green) to start the program

## start ROS on your machine
Assuming the dependencies are installed
1. start robot interface streaming
```
roslaunch fanuc_cr35ia_support robot_interface_streaming_cr35ia.launch robot_ip:=192.168.1.121
```

Comments: DON'T set ROS_MASTER_URI in the ~/.bashrc

2. check ros topic
```
rostopic list
```

3. send a trajectory command
```
rostopic pub -1 /joint_path_command trajectory_msgs/JointTrajectory "header:
...
joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']"
points:
- position:[0.2,0,0,0,0,0]
  velocities:[0,0,0,0,0,0]
  accelerations:[0,0,0,0,0,0]
  effort:[0,0,0,0,0,0]
...
```

4. start the trajectory
```
cd catkin_ws/src
git clone https://github.com/suneric/fanuc_ros_control.git
roslaunch fanuc_ros_control trajectory.launch
```

## Fanuc CR35ia joint limit
[Notes](http://wiki.ros.org/fanuc_cr35ia_support):
There is currently some confusion over the correct values for the joint limits of joints 2 and 3. Version B-83734EN/01 of the Operator's Manual incorrectly states that 1.05 rad equals 120 degrees, and that 0.39 rad equals 45 degrees (joint 2). Additionally, it gives a value of -122.9 degrees for the lower limit of joint 3, whereas (at least) Roboguide Rev K have this limit set to -182 degrees.

|joint|min|max|
|:---:|:---:|:---:|
|joint 1|-2.9670|2.9670|
|joint 2|-0.7853|2.0943|
|joint 3|-2.1450|2.3561|
|joint 4|-3.4906|3.4906|
|joint 5|-1.9198|1.9198|
|joint 6|-7.8539|7.8539|

information found on pendant (degree)

|joint|min|max|
|:---:|:---:|:---:|
|joint 1|-185.00|185.00|
|joint 2|-45.00|120.00|
|joint 3|-122.96|135.00|
|joint 4|-200.00|200.00|
|joint 5|-110.00|50.00|
|joint 6|-450.00|450.00|


[reference 1](https://github.com/ros-industrial/fanuc/blob/melodic-devel/fanuc_cr35ia_support/urdf/cr35ia_macro.xacro), and [reference 2](https://www.fanuc.co.jp/en/product/catalog/pdf/robot/RCR-35iA(E)-02a.pdf)


### Safety
1. DCS zones, a DCS zone is configured to be a virtual "fence" using the Cartesian position check. A DCS zone is configued for Joint 1 of the robot to be capable of working within +/- 85 degree. If joint 1 exceeds these limits, the robot will perform a stop type of Category 0.

2. joint 2 (-45 degree ~ +58 degree), considered the limited space on upper of the room, limit joint 2 to [0, 45] degree
3. joint 3 (-116 degree ~ 135 degree)

Recommanded Joint Limit
|joint|min|max|
|:---:|:---:|:---:|
|joint 1|-1.4835|0.3751|
|joint 2|-0.0150|0.7853|
|joint 3|-1.0472|0.0000|
|joint 4|-3.4906|3.4906|
|joint 5|-1.9198|0.8726|
|joint 6|-7.8539|7.8539|
