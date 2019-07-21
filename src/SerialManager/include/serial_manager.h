#include "ros/ros.h"
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <strings.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <queue>
#include <sys/time.h>


#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

/************HEADERS NEEDED FOR THE STATE MACHINE***************************************************/

#define HEADER_BYTES  2
#define HEADER_A  26
#define HEADER_B 27
#define PAYLOAD_POSE 44
#define PAYLOAD_POSE_R 45

using std::cout;
using std::endl;


typedef enum{
    HEADER_1,
    HEADER_2,
    ID,
    wait_PAYLOAD,
} waiting_msg;
waiting_msg state_msg;


/************BUFFER DI RICEZIONE E TRASMISSIONE***************************************************/
//buffer di ricezione
std::queue<unsigned char> coda_recv_seriale;
//bufferi di trasmissione
std::queue<unsigned char> coda_send_seriale;
std::string seriale_dev;


/******************GLOBAL VAR***********************************************************************/
int offset = 0;
int new_packet = 0;
double param = 0.0;
char new_packet_pose = 0;
int pose_el_time;
double pos_x_f=0 ;
double pos_y_f=0;
double pos_z_f=0 ;
double pos_x_b=0 ;
double pos_y_b=0 ;
double pos_z_b=0 ;
double way_x=0 ;
double way_y=0 ;
double way_z=0 ;
double orient_;
bool stream_pose;
double startstop=0;
int serial;

geometry_msgs::Point h_angle;

//strutture dati emporali
timeval current_time, stream_pose_time;
double elapsed_time_pose;
/******************READ FROM SERIAL***********************************************************************/
void decode_packet();
double decode_payload();
void read_from_serial(int* serial);
/******************WRITE TO SERIAL***********************************************************************/
void encode_payload(double payload);
void write_to_serial(int* serial); 
void parser_mess(unsigned char buffer);
int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);
int serial_init(int* fd,const char* seriale_dev);

bool isDifferent(double* , double*,double* , double* ,int* );

/******************SUBSCRIBER CALLBACKS***********************************************************************/

void Pose_cb_forward(const geometry_msgs::Point::ConstPtr& msg);
void Pose_cb_back(const geometry_msgs::Point::ConstPtr& msg);
void start_and_stop_cb(const std_msgs::Float64::ConstPtr& msg);
void waypoint_cb(const geometry_msgs::Point::ConstPtr& msg);

