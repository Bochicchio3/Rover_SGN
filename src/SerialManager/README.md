# SerialManager

This package handles the communication between the STM and the Intel Joule.

It consist in:
1. SerialManager/include/serialmanager.h
2. SerialManager/src/serialmanager_function_B.cpp
3. SerialManager/src/serialmanager_node_B.cpp

## serialmanager.h

Structure:

1. Headers for the state machine

2. Declare write and read buffer

3. Declare global variable

4. Read from serial

5. Write to serial

6. Subscriber Callbacks

## serial_manager_function_B.cpp

Structure and functions:

1. Callback functions

1.1 void Pose_cb_forward

Subscribe to the forward tag pose topic and assign to pos_[X/Y/Z]_f the content of the message which will be published through serial.

1.2 void Pose_cb_back

Subscribe to the backward tag pose topic and assign to pos_[X/Y/Z]_b the content of the message which will be published through serial.

1.3 void start_and_stop_cb

Subscribe to the start_and_stop topic and assign to startandstop the content of the message which will be published through serial.

1.4 void waypoint_cb

Subscribe to the waypoint topic and assign to way_[X/Y/Z] the content of the message which will be published through serial.

2. set_interface_attribs && set_blocking && serial_init

Do not modify unless you need to change the serial protocol. Here you can find information about the serial port, baud rate, ecc.

3. write_to_serial && encode_payload

Serial can only handle Uint8. Encode paylode manually convert doubles to arrays of chars.
#### Common pitfalls: the conversion order on the Intel Joule and STM MUST match, otherwise you will make mistakes when you recover the double from the array of chars.
write_to_serial writes the first element of the conda_send_seriale buffer and then pops it out.

4. parser_mess && read_from_serial && decode_payload && decode_packet

This functions are responsible for reading from the serial protocol.
read_from_serial takes batches of 1024 chars from serial and send them to the state machine parser_mess
parser_mess decodes the packets. The structure of the packet must be:

- HEADER_A
- HEADER_B
- PAYLOAD_POSE
- PAYLOAD 

You can change the dimension of the payload modifying N (current value is 4) in: 
```
case wait_PAYLOAD:
      if(offset<N)
```
N is the number of bytes, so if you want to send a Double it should be 4. We are only sending the orientation from the filter implemented in the low level controller, so we just need a Double.
If the message is correctly received and has the right structure, the state machine should sucesfully update the conda_recv_seriale buffer:

```
  coda_recv_seriale.push(buffer);
  state_msg=wait_PAYLOAD;
  offset++;
```
and eventually increment the packet counter:
  ```
  new_packet++;
  ```          
  
Many things can go wrong with the state machine, in such a case, debug with the ROS_INFO and ROS_DEBUG tools by printing which states you access.  

Decode packet and decode paylode 

## serial_manager_node_B.cpp

This is the main executable. 
Structure:
1. Publisher and Subscriber
Here you can modify topics names.
2. Parameter for the serial port
Do not modify unless you make specific changes to the serial protocol.
3. While ros::ok() loop

  3.1 Read serial
  ```
  read_from_serial(&serial);
  h_angle.z=orient_;
  heading_angle.publish(h_angle);
  ```
  3.2 Write to serial
  ```
    coda_send_seriale.push(HEADER_A);
    coda_send_seriale.push(HEADER_B);
    coda_send_seriale.push(PAYLOAD_POSE);
    encode_payload(pos_x_f);
    encode_payload(pos_y_f);
    encode_payload(pos_z_f);
    encode_payload(pos_x_b);
    encode_payload(pos_y_b);
    encode_payload(pos_z_b);
    encode_payload(way_x);
    encode_payload(way_y);
    encode_payload(way_z );
    encode_payload(startstop);
    new_packet_pose = 0;
    write_to_serial(&serial);
  ```
  
### Usefull hints 
  
It is really usefull to debug to verify that everything is working correctly. It can be very difficoult to spot the error within the state machine.
A really usefull tool to debug realtime is:
 ```
   ROS_INFO(" Some information %f ", double variable name );
```
