# ROS Package: espeleo_bringup
Package with the Espeleorobot startup scripts

## Installation

#### Add espeleo to dialout to prevent connection problems with the USB Arduino
```
sudo adduser $USER dialout
```

#### Define robot hostname
```
sudo vim /etc/hostname
    espeleo-robo
```

#### Create a file to set the environment variables for ROS framework
- such as hostname and master URI
```
sudo mkdir -p /etc/itv
sudo ln -s $(rospack find espeleo_bringup)/service_files/itv_env.sh /etc/itv/env.sh
sudo chmod 755 /etc/itv/env.sh
```

#### Create the roscore service and enable it to start at boot
```
sudo systemctl enable $(rospack find espeleo_bringup)/service_files/roscore.service
```

#### Create the service to start the CAN0 interface
```
sudo systemctl enable $(rospack find espeleo_bringup)/service_files/can0_bringup.service
```

#### Create the espeleo service to start all required nodes for the robot
```
sudo systemctl enable $(rospack find espeleo_bringup)/service_files/espeleo_ros.service 
```
#### Configure the espeleo start script with LOG file and ROS nodes
```
sudo ln -s $(rospack find espeleo_bringup)/service_files/espeleo_ros_start /usr/sbin/espeleo_ros_start
sudo chmod 755 /usr/sbin/espeleo_ros_start
