## ROS package to control espeleo with joystick or keyboard

#### Install

- clone package into ros src workspace
```
git clone https://github.com/ITVRoC/espeleo_teleop.git
```

- verify if all ros dependencies are installed
```
rosdep install --from-paths src --ignore-src -r -y
```

- compile the workspace
```
catkin_make
```

Click [here](https://github.com/ITVRoC/espeleo/wiki/Joystick-settings) to check the guide on how to config a linux input joystick and its permissions.

