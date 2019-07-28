# Icaro UWB

This is a python package for ros that enables the localization of the Rover using the POSZYX system.

The original reference for the python code is the work of Crosato Tesconi carried out at University of Pisa.
For a detalied explaination of the code: 

### To do: ask permission for the repository

Only small modifications have been made to switch from a python executable to a Ros package.
Here you will find a detailed explanation of this modfications.


## 1. Anchors ID and coordinates

This is where you must upload the coordinates resulting from auto calibration or manual calibration
Make sure that anchor_ids are correct. 
BE AWARE that some of the anchor may not work. The reliable anchors are: 0x6902, 0x6e7a, 0x6e44, 0x6e6c
```
anchor_ids = [0x6902, 0x6e7a, 0x6e44, 0x6e6c]
anchors = [DeviceCoordinates(anchor_ids[0], 1, Coordinates(0, 0, 0)),
       DeviceCoordinates(anchor_ids[1], 1, Coordinates(0,5228.0  , 0)),
       DeviceCoordinates(anchor_ids[2], 1, Coordinates(3251, 354, 0)),
       DeviceCoordinates(anchor_ids[3], 1, Coordinates(158, -40, 1326))]
```

## 2. Serial ports configuration

```
Set serial port for detecting the tag
serial_port = '/dev/ttyACM' + tag_id
```
Default tag_id are 0 and 1, There shouldn't be any problem with the serial port id. If you get an error message and the roslaunch fails, verify by listing devices on your computer that the tags are connected to the proper ports.

## 3. Publisher
```
if status == POZYX_SUCCESS:
    if (self.id == 0):
        rospy.loginfo("id tag: " + str(self.id))
        print(str(position.x)+'\t'+str(position.y)+'\t'+str(position.z)+'\t')
    self.Pointmsg.x=position.x
    self.Pointmsg.y=position.y
    self.Pointmsg.z=position.z
    if (self.Pointmsg.x is not 0) and (self.Pointmsg.y is not 0) and (self.Pointmsg.z is not 0):
        self.pub.publish(self.Pointmsg)
```

Publish the position message in the topic determined by the tag id (0 or 1 by default)

