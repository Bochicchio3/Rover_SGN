//=================================================================================================
//
//   nodo ROS per leggere e scrivere su seriale
//
//=================================================================================================
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
//#include "serial_manager/Param.h"
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <queue>
#include <sys/time.h>
//#include <aruco_mapping/ArucoMarker.h>
//#include <geometry_msgs/Point.h>
//#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "obstacle_avoidance/distance_msg.h"



/*************************************************************/


// PACCHETTI IN TRASMISSIONE
//1- pos packet
//     _____________________________________________________________________________________
//     | HEADER_CMD_B |HEADER_CMD_A|  PAYLOAD_POS   | X_POS  | Y_POS | Z_POS | ROLL | PITCH | YAW |
//     _____________________________________________________________________________________

/*define bytes di header */
#define HEADER_BYTES  2
#define HEADER_A  (int)0x1A
#define HEADER_B (int)0x1B
//#define PAYLOAD_PING (int)0x1C
#define PAYLOAD_POSE (int)0x2C
#define PAYLOAD_POSE_R (int)0x2D

/**************************************************************************************************/
/*********************************************MACCHINA A STATI*************************************/
/**************************************************************************************************/
typedef enum{
    HEADER_1,
    HEADER_2,
    ID,
    wait_PAYLOAD,
} waiting_msg;

//variabile per la memorizzazione dello stato della macchina a stati
waiting_msg state_msg;
/**************************************************************************************************/
/******************PACCHETTO STM************************************************************/
/**************************************************************************************************/
//#define N_BYTE 24
#define N_BYTE    228
#define N_NUMBERS 57


/************BUFFER DI RICEZIONE E TRASMISSIONE***************************************************/
//buffer di ricezione
std::queue<unsigned char> coda_recv_seriale;
//bufferi di trasmissione
std::queue<unsigned char> coda_send_seriale;

/**********************TOPIC ROS********************************************************************/
//ros Subscriber
//ros::Subscriber pose_topic;

//aggiunto da Barbara

//ros Subscriber
ros::Subscriber distance_topic;
//riga ros topic request scommentata da Barbara

//ros topic request

//ros::Publisher stm_pose_topic;
//ros::Publisher rviz_pose_topic;

//aggiunto da Barbara
ros::Publisher stm_distance_topic;


//aggiunto da Ale 
double pos_x ;
double pos_y ;
double pos_z ;
double roll_s ;
double pitch_s ;
double yaw_s ;

double new_grafic_stm[N_BYTE+1];


/******************GLOBAL VAR***********************************************************************/
int offset = 0;
int new_packet = 0;
double param = 0.0;
char new_packet_pose = 0;
int pose_el_time;
//double PI = 3.14159;
using std::cout;
using std::endl;

//strutture dati emporali
timeval current_time, stream_pose_time;
double elapsed_time_pose;

//struttura per la memorizzazione della posa della camera nel frame world
//struct global_pose
//agg da Barbara
struct distance_pose
{
    //geometry_msgs::Point position;
    //geometry_msgs::Point orientation;

    //aggiunto da Barbara
      obstacle_avoidance::distance_msg distance;

};
//global_pose global_camera_pose;

//aggiunto da Barbara
//distance_pose distance_object_pose;
   distance_pose grafic; //cambiato 12/03
//geometry_msgs::PoseStamped stm_Pose;
//geometry_msgs::PoseStamped rviz_Pose;

//aggiunto da Ale
geometry_msgs::Pose stm_Pose;

//aggiunto da Barbara 
obstacle_avoidance::distance_msg stm_distance;

bool stream_pose;


int serial;

/***************************FUNZIONI************************************************************************/
void encode_payload(double payload);
int write_to_serial(int* serial);
//void read_from_serial(int* serial);
//void Pose_cb(const aruco_mapping::ArucoMarker::ConstPtr& msg);
void Pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

//aggiunto da Barbara
void distance_cb(const obstacle_avoidance::distance_msg::ConstPtr& msg);

//aggiunto da Ale 
bool isDifferent(double* , double*,double* , double* ,int* );


void parser_mess(unsigned char buffer);
double decode_payload();
//void decode_packet();
int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);
int serial_init(int* fd,const char* seriale_dev);
void quaternion_2_euler(double xquat, double yquat, double zquat, double wquat, double& roll, double& pitch, double& yaw);
//void euler_2_quaternion(double& xquat, double& yquat, double& zquat, double& wquat, double roll, double pitch, double yaw);
