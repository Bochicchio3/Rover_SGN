# Rover_SGN

This is the main package of the Rover project.
It consists in two subpackages which are:

1. SerialManager, which handles the bidirectional communication based on the serial protocol. It is responsible for the data exchange between the Intel Joule and the STM32. 

2. Icaro_uwb, essential to enable the ultrawideband localization feature. It interacts with the two tags on the rover and with the four anchors and publish the tag_0 and tag_1 poses in the right topics.

For details on this two packages navigate to their directories and read the readme.
Here you will find the general guide to setup the whole system.

## System Setup

To setup everything on the Intel Joule:

```
cd
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
catkin init
git clone https://github.com/Bochicchio3/Rover_SGN/
git clone  https://github.com/Bochicchio3/RoverGUI
cd ..
catkin build
source devel/setup.bash
```
or
```
echo source devel/setup.bash >> ~/.bashrc
```
to source it every time you open a terminal.


## Remote connection

By default, the Intel Joule, will automatically setup a ROSNET local wifi after boot. This is done to allow remote connection to the Joule.

You will need a PC with Ubuntu and Matlab to operate with the Rover remotely via SSH. Any version of matlab between 2014a and 2018b should be fine. For Matlab on Ubuntu, just login in your mathworks account and follow the procedure for matlab on Ubuntu.
After downloading Matlab and navigating to the download directory:
```
sudo ./install
```
Follow the procedure, you only need to install Matlab, GUIDE toolbox and Robotics Control Toolbox.
After installed, I suggest to make an alias for matlab:

```
gedit  ~/.bashrc
```
and add the following line at the end of the document where you insert your Matlab version instead of [matlab_version]:
```
alias matlab= cd ../../usr/local/MATLAB/[matlab_version]/bin && ./matlab
```
Save with Ctrl+s.
Now either open a new terminal or source the workspace again:
```
source ~/.bashrc
matlab
```
Matlab should be opening now. Follow the guide on the official Matlab_Gui repository to run the graphical interface.


## Launch the software from the Remote PC
Before starting the procedure, make sure that you are either connected to ROSNET or connected to the same local network as the Intel Joule.
I suggest to install terminator, for its multiwindow capability which is very handy when you have open multiple terminals.
On the Remote Pc, open a new instance of Terminator. Press F11 to switch to fullscreen mode. Create multiple terminals with Ctrl+shift+E and Ctrl+shift+O for new orizontal and vertical terminals.
You will need 4 terminals.
On each terminal:
```
ssh -XC robot@192.168.20.1
```
The password is: robot
The -XC command is required to share the graphical interface between the connected computers ( in our case, RoverGui)
You may also use an alias for the connection:
```
gedit  ~/.bashrc
```
add the following line at the end of the document
```
alias connect_rover=ssh -XC robot@192.168.20.1
```
then source ~./bashrc or open a new terminal and just type: 
```
connect_rover
```
The password is: robot

BE AWARE: the ip_address may change, so make sure that you are connected to the right ip_address.

You need to have at least three terminals connected to the Rover.
Usefull hint: you can navigate within the terminals with Alt + Left/Right/Up/Down arrow

### In the first terminal

```
roslaunch icaro_uwb (TAB for auto completion)
```
### In the second terminal:

```
roslaunch serial_manager (TAB for auto completion)
```
### In the third terminal:
```
rosrun rover_gui (TAB for auto completion)
```

## THIS WILL NEED TO BE UPDATED AFTER THE REVIEW IS COMPLETED

Now you should see the Rover_gui interface and the Matlab_gui opened.
You can succesfully proceed to command the Rover to the desired waypoint.
Read the guide on the Matlab_GUI for details on how to control the robot.


