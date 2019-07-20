#include "serial_manager.h"


#include "obstacle_avoidance/distance_msg.h"

// aggiunto da Barbara
double new_grafic[57];


void distance_cb(const obstacle_avoidance::distance_msg::ConstPtr& msg){
      
       for(int i=0; i<57;i++){
 	   new_grafic[i] = (double) msg->grafic[i];
       }                          
        new_packet_pose = 1;
}


obstacle_avoidance::distance_msg distance_msg;

/*****************************************************************/
/*                                                               */
/*                 MAIN                                          */
/*****************************************************************/
int main(int argc, char **argv)
{
	  
    ros::init(argc, argv, "Serial_Manager");

    ros::NodeHandle n;

    distance_topic = n.subscribe<obstacle_avoidance::distance_msg>("/distances", 1, &distance_cb);
    stm_distance_topic = n.advertise<geometry_msgs::Pose>("distance_topic_from_object",100);


    //leggo i parametri specificati nel launch file
    std::string seriale_dev;
    n.param<std::string>("/SerialManager/dev", seriale_dev, "/dev/ttyS1");
    n.param<bool>("/SerialManager/stream_pose", stream_pose, false);
    n.param<int>("/SerialManager/pose_el_time", pose_el_time, 500);
        
    //DEBUG
    double a,b;
    int p;
    //

    int serial;

    // init della seriale
    int result = serial_init(&serial, seriale_dev.c_str());


    ros::Rate loop_rate(100); // 100 Hz
    if (result == 1){
          //ROS_INFO_STREAM("Serial configuration Ok, now ROS  "  );   
        while(ros::ok()){


              //LEGGO DALLA SERIALE
	           read_from_serial(&serial);
	              
                  if(isDifferent(&new_grafic[0] , &new_grafic_stm[0],&a,&b,&p) ){

                   
                   ROS_INFO("-----------------------------\n"  );
                   ROS_INFO("WARNING: DIFFERENT VALUES    \n"  );
                   ROS_INFO("A=%f  B=%f  and position P=%d\n",a,b,p );
                   ROS_INFO("-----------------------------\n"  );
                   
                   
			      }					  
     
                  if(new_packet_pose){ 
                     //ROS_INFO_STREAM("Arrived new packet from distance_topic_from_object  "  );                       
                     coda_send_seriale.push(HEADER_A);
                     coda_send_seriale.push(HEADER_B);
                     coda_send_seriale.push(PAYLOAD_POSE);
                     for(int i=0;i<57;i++)
                          encode_payload(new_grafic[i]);
                     
                    write_to_serial(&serial);    
                    new_packet_pose = 0;
                    //ROS_INFO_STREAM("... and writed to serial "  ); 
              }
           
          //vedi se arrivato qualcosa sulle callback
          ros::spinOnce();
          
        
         loop_rate.sleep();
     }//END WHILE

    }else {
	     ROS_INFO("WARNING !!!!! Serial configuration problems !!! ");
    }


  return 0;
}
