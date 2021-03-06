# README UFMG

# EspeleoRobô CoppeliaSim/ROS Simulator
----------------------
This repository contains the models to simulate EspeleoRobô in CoppeliaSim (the old V-REP).

## Firs step:

Make sure you have CoppeliaSim properly installed. Please go to the README of the package https://github.com/ITVRoC/sim_ros_interface. Follow the instructions therein come back here.

**Test if everything is working:**

Clone these packages in your catkin workspace `src` (example: `~/catkin_ws/src`) folder:

- espeleo_control: `git clone https://github.com/ITVRoC/espeleo_control.git`
- espeleo_decawave: `git clone https://github.com/ITVRoC/espeleo_decawave.git`
- espeleo_localization: `git clone https://github.com/ITVRoC/espeleo_localization.git`
- espeleo_planning: `git clone https://github.com/ITVRoC/espeleo_planning.git`
- imu_complemenntary_fiter: `git clone https://github.com/ITVRoC/imu_complementary_filter.git`
- ros_eposmcd (note: this should already installed installed): `git clone https://github.com/ITVRoC/ros_eposmcd.git`

Compile:

`catkin build`

Resource your workspace:

`source ~/catkin_ws/devel/setup.bash`

After running rosmaster (`roscore`), go to your `src` folder and run CoppeliaSim with a defined scene:

`coppeliasim ~/catkin_ws/src/espeleo_vrep_simulation/vrep_models/scenarios/terrain_tree/terrain_tree.ttt`

Play the simulator ad run a simple keyboard controller to check if everything is working:

`roslaunch espeleo_vrep_simulation keyboard.launch`

Note: Replace `~/catkin_ws` if you have a different path to your catkin workspace.

## EspeleoRobô models included:
- 6 legs
- 6 wheels
- 6 wheels plus imu, kinect and velodyne
- 6 wheels plus imu, kinect and hokuyo spinning
- 6 star shaped wheels
- 4 wheels and 2 legs
- 4 wheels 2 legs plus imu and kinect
- 4 wheels 2 legs plus imu and velodyne

## Scenes included:
- tube (cylinder) plus espeleorobo
- terrain plus tree plus espeleorobo
- paleotoca
- campinho (Motocross field)
- DARPA SubT Challenge scene

## List of nodes

- `imu_basic_node.py` This node captures information from the gyro and the accelerometer and publishes a imu raw topic. OBS: the complementary filter in the package `imu_complementary_filter` should also be used to generate the quaternion data of the IMU.

- `pose_constructor_sim.cpp` This node publishes a pose topic of the espeleorobo. It is based on the and on the data from the simulated imu and from the `\tf`. This is an outdated code. Use `state_estimator_espeleo.cpp` from the package `espeleo_localization` instead. OBS: the complementary filter in the package `imu_complementary_filter` should also be used to generate the quaternion data of the IMU.

- `xsens_emulator.py` This node simulates the imu and the GPS data of the real espeleorobo. It is a better version of the node above node. If your code works only with `imu_basic_node.py` (togueger with `\tf` data) you should update it to work with the `xsens_emulator.py` node, since it better replicates the real robot. This node receives three parameters for setting the initial global position of the xsens, they are: (i) LAT in degrees, (ii) LON in degrees, (iii) ALT in meters above sealevel.
OBS: the complementary filter in the package `imu_complementary_filter` should also be used to generate the quaternion data of the IMU.

- `config_rviz_basic.py` Simple script to configure rviz

- `config_rviz_cylinder.py` Simple script to configure rviz in the solid cylinder scenario


## List of launch files

Note: Before running these launch files it is necessary to run ROS, then vrep and load the desired scene.

- `basic.launch` Launch the codes to control the espeleorobo with the vector field approach. After running this the controller will wait for a path to be followed

- `keyboard.launch` Launch the codes to control the espeleorobo with the keyboard


## How to interact

The simulated models behave like the real robot, emulating its topics and services in ROS.

**Topics:**
- `/ros_eposmcd/position_movement_absolute`  (message type:`ros_eposmcd_msgs/movement`): Sends relative position commands to a chosen motor in **[rad]**.
- `/ros_eposmcd/position_movement_relative`  (message type:`ros_eposmcd_msgs/movement`): Sends absolute position commands to a chosen motor in **[rad]**.
- `/ros_eposmcd/velocity_movement`  (message type:`ros_eposmcd_msgs/movement`): Sends velocity commands to a chosen motor in **[rad/s]**.

- `/sensors/acc`  (message type:`geometry_msgs/Point`): Data from the accelerometer simulated inside vrep.
- `/sensors/gyro`  (message type:`geometry_msgs/Point`): Data from the gyro simulated inside vrep.
- `/sensors/velodyne`  (message type:`sensor_msgs/PointCloud`): Data from the ouster laser simulated inside vrep.
- `/imu/raw`  (message type:`sensor_msgs/Imu`): Message that simulates a imu raw data (accelerometer and gyro)
- `/imu/data`  (message type:`sensor_msgs/Imu`): Message that simulates a imu data, with orientation quaternion
- `/cmd_vel`  (message type:`geometry_msgs/Twist`): Velocity commands for the robot, linear forward velocity and angular velocity around z axis.
- `/espeleo/pose`  (message type:`geometry_msgs/Pose`): Message that contains a pose estimation for the espeleorobo


**Services**
- `/vrep_ros_interface/ros_eposmcd/request_position`  (message type:`ros_eposmcd_msgs/maxon_telemetry`): Request a desired motor angular position in **[rad]**.
- `/vrep_ros_interface/ros_eposmcd/request_velocity`  (message type:`ros_eposmcd_msgs/maxon_telemetry`): Request a desired motor velocity in **[rad/s]**.


## Contact

Any questions, please contact-me in ``adrianomcr18@gmail.com``.

-----------
# README espeleo2_vrep

# EspeleoRobo V-REP/ROS Simulator
----------------------
This repository contains all needed files for simulating EspeleoRobo in V-REP.

## EspeleoRobo models included:
- 6 legs
- 6 wheels
- 6 star shaped wheels
- 4 wheels and 2 legs
- 4 wheels 2 legs plus imu and kinect
- 4 wheels 2 legs plus imu and velodyne

## Scenes included:
- tube (cylinder) plus espeleorobo
- terrain plus tree plus espeleorobo
- paleotoca
- campinho (Motocross field)

## How to Install CoppeliaSim with ROS Bridge

Install Tutorial Coppeliasim - Espeleo Simulation

- 1 - Download CoppeliaSim (CoppeliaSim Edu 18.04 or 16.04, according to your linux version) - https://coppeliarobotics.com/downloads . Unzip in a suitable folder.

- 2 - Add a line on your .bashrc, indicating your CoppeliaSim root path (Path from previous step):


		$ export COPPELIASIM_ROOT_DIR="<path_to_coppeliasim_root_dir>"
	
	
	Save, close e reload the .bashrc:
	
	
		$ source ~/.bashrc

- 3 - Create an alias of the simulator to your terminal:


```sh
	$ echo "alias coppelia=$COPPELIASIM_ROOT_DIR/coppeliaSim.sh" >> ~/.bashrc
	$ source ~/.bashrc
```

- 4 - Test if the program is working on terminal by:


		$ coppelia

- 5 - Go to your "../catkin_ws/src/" and clone recursively the plugin repository:


		$ git clone --recursive https://github.com/CoppeliaRobotics/simExtROSInterface.git sim_ros_interface
		

- 6 - Install the support packages:


		$ sudo apt-get install python-catkin-tools xsltproc ros-$ROS_DISTRO-brics-actuator ros-$ROS_DISTRO-tf2-sensor-msgs
		

- 7 - Use "catkin build" to compile your packages. To do so, you must "catkin clean", then "catkin build"

```sh
	$ catkin clean		
	$ catkin build
```

- 8 - If your compilation finished succesfully, the library "libv_repExtRosInterface.so" compiled correctly. 
	This library makes CoppeliaSim recognize the ROS enviroment in your machine. Now, copy this library to the CoppeliaSim directory:
	
	
		$ cp ~/catkin_ws/devel/lib/libsimExtROSInterface.so $COPPELIASIM_ROOT_DIR
		
		
- 9 - Everything is ready to run. To test the communication, run the ROS master:


		$ roscore

- 10 - Now run CoppeliaSim:


		$ coppelia
		
		
Note that there are multiple init messages from CoppeliaSim on terminal. An indication that your library was compiled correctly is the following message:


```
		Plugin 'RosInterface': loading...
		Plugin 'RosInterface': warning: replaced variable 'simROS'
		Plugin 'RosInterface': load succeeded.
```


- 11 - To confirm the interaction between ROS and CoppeliaSim, play the empty scene in the begin of the program. In other terminal type:


		$ rostopic list
		
		
	If the topic "/tf" appears, the ROS/CoppeliaSim is enabled and functional.
	
	

## Final Considerations:

The **libv_repExtRosInterface.so**, by default, only accepts common ROS message types in CoppeliaSim. In case you wish to use other message types or even custom messages, do as indicated in the interface repository:
	 	
		
> Edit meta/messages.txt and meta/services.txt if you need to include more ROS > 
> messages/services. You need to specify the full message/service type, i.e. 
> geometry_msgs/Twist rather than Twist.

	
These files are inside the interface package. Besides this, it is necessary to add the message package dependency in  th CMakelists.txt and package.xml.

After the edition of these files to add new elements to the library, a new recompilation and copy of the library must be done. So every time you add a new message, this process must be repeated.
	
## Troubleshooting

- If you got the following error:

```
print('error: program "{0}" is missing (hint: try "sudo apt install {0}")'.format(what), file=sys.stderr)
```

Change the folder `programming/libPlugin` from your COPPELIASIM_ROOT_DIR for the following [libPlugin](https://github.com/CoppeliaRobotics/libPlugin) and compile again:


```
$ cd $COPPELIASIM_ROOT_DIR/programming
$ sudo rm -r libPlugin
$ git clone https://github.com/CoppeliaRobotics/libPlugin.git
```

# Espeleo Simulation


To run the EspeleoRobo Coppelia simulation, first, clone this repository and other packages needed in your workspace:
```sh
$ git clone https://github.com/ITVRoC/espeleo_vrep_simulation.git
$ git clone https://github.com/ITVRoC/espeleo_locomotion.git
$ git clone https://github.com/ITVRoC/espeleo_description.git
$ git clone https://github.com/ITVRoC/espeleo_bringup.git
$ git clone https://github.com/ITVRoC/espeleo_msg_srv.git
```
Also clone or install via apt the ROS Web Video Server:
```sh
sudo apt install ros-<distro>-web-video-server (RECOMMENDED)
or
git clone https://github.com/RobotWebTools/web_video_server.git
```

### Espeleo Locomotion
The espeleo_locomotion package is responsible for control the movement of the robot, sending the RPM for each wheel, according to the kinematic model used. This package is also responsible for loading some mechanical parameters used in the simulation, like the value of each wheel reduction.   The simulation will not run without those parameters.

### Espeleo Description
Espeleo description package is responsible for the EspeleoRobo TF tree. The simulation only provides the frames of each sensor and the relation of the "base_link" frame to the "world" frame. 

### Espeleo Bringup
Espeleo bringup is responsible to start the dynamic reconfigure server, allowing the use of some functionalities like "turbo button" and changing the direction of the movement. 

### Espeleo Msg and Srv
Espeleo messages and services contains espeleo's messages and services data structures.

### Web Video Server
Required package to convert ROS Streams to HTTP, allowing to use the robot's cameras in simulation in Espeleo's GUI.

## Optional packages

### Espeleo GUI

Espeleo_gui is a control interface that allows the user to get the motor's current feedback, front and back camera streams, record videos, bags and change some parameters of EspeleoRobo. 
```sh
$ git clone https://github.com/ITVRoC/espeleo_gui.git
```

### Espeleo Teleop
 
This package has nodes especially adapted that allow the use of a keyboard or joystick to control EspeleoRobo. 
```sh
$ git clone https://github.com/ITVRoC/espeleo_teleop.git
```

## How to run
After compiling all the needed packages, go to espeleo_vrep_simulation and change the branch to espeleo2_vrep and compile the package:
```sh
$ roscd espeleo_vrep_simulation
$ git checkout espeleo2_vrep
$ catkin build espeleo_vrep_simulation
```

Then, use these commands to run the simulation:

```sh
$ roscore
$ coppelia
```

In Coppelia, open the  EspeleoRobo scene or model. Select "**File**" and "**Open new scene**".

Search for "**_espeleo_6wheel_ITV.ttt_**", located in Folder : "**espeleo_vrep_simulation/vrep_models/scenarios/ITVRoC/**

Then:

```sh
$ roslaunch espeleo_vrep_simulation espeleo_sim.launch
```

 And press play in the simulation. 
 
 You can launch the GUI with the following command:
 ```sh
$ roslaunch espeleo_gui gui_simulation.launch
```


## Contact

Any questions, please contact-me in ``mateusnazarioc@gmail.com``.
All pull requests are welcome and desired!!!
