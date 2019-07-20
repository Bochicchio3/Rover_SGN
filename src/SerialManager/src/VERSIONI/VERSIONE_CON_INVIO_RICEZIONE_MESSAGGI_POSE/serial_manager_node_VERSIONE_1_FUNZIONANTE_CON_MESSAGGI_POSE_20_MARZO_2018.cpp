#include "serial_manager.h"


#include "obstacle_avoidance/distance_msg.h"

// aggiunto da Barbara

double new_grafic[57];

void distance_cb(const obstacle_avoidance::distance_msg::ConstPtr& msg){
      
       for(int i=0; i<57;i++){
 	   new_grafic[i] = (double) msg->grafic[i];
       }
        
         //ROS_INFO_STREAM("1 element: " << new_grafic[0]);   
         //7ROS_INFO_STREAM("2 element: " << new_grafic[1]);  
         //ROS_INFO_STREAM("3 element: " << new_grafic[2]);   
         //ROS_INFO_STREAM("4 element: " << new_grafic[3]);
         //ROS_INFO_STREAM("5 element: " << new_grafic[4]);   
         //ROS_INFO_STREAM("6 element: " << new_grafic[5]);
           
         
               
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
    //distance_topic = n.subscribe<obstacle_avoidance::distance_msg>("/distances", 1000, &distance_cb);
    //      //ros::Publisher stm_distance_topic = n.advertise<obstacle_avoidance::distance_msg>("stm/distance_topic_from_object",100);
    //ros::Publisher stm_distance_topic = n.advertise<obstacle_avoidance::distance_msg>("distance_topic_from_object",100);
    // aggiunto da Ale 
    stm_distance_topic = n.advertise<geometry_msgs::Pose>("distance_topic_from_object",100);


    //leggo i parametri specificati nel launch file
    std::string seriale_dev;
    n.param<std::string>("/SerialManager/dev", seriale_dev, "/dev/ttyS1");
    n.param<bool>("/SerialManager/stream_pose", stream_pose, false);
    n.param<int>("/SerialManager/pose_el_time", pose_el_time, 500);
        
   

    int serial;

    // init della seriale
    int result = serial_init(&serial, seriale_dev.c_str());



				//inizializzo stream_pose_time

				//gettimeofday(&stream_pose_time, NULL);
    ros::Rate loop_rate(100); // 100 Hz
    if (result == 1){
          //ROS_INFO_STREAM("Serial configuration Ok, now ROS  "  );   
        while(ros::ok()){


              //LEGGO DALLA SERIALE
	           read_from_serial(&serial);
	              


          
                  if(new_packet_pose){ 
                     //ROS_INFO_STREAM("Arrived new packet from distance_topic_from_object  "  ); 
                      
                     coda_send_seriale.push(HEADER_A);
                     coda_send_seriale.push(HEADER_B);
                     coda_send_seriale.push(PAYLOAD_POSE);
                     encode_payload(new_grafic[0]);
                     encode_payload(new_grafic[1]);
                     encode_payload(new_grafic[2]);
                     encode_payload(new_grafic[3]);
                     encode_payload(new_grafic[4]);
                     encode_payload(new_grafic[5]);




                          
                     //for(int i=0; i<49;i+=6){
                     //
                     //       coda_send_seriale.push(HEADER_A);
                     //       coda_send_seriale.push(HEADER_B);
                     //       coda_send_seriale.push(PAYLOAD_POSE);
                     //       for(int j=0;j<6;j++){
                     //          encode_payload(new_grafic[i+j]);
                     //       }
                     //  ROS_INFO_STREAM("Frame packet num = " << i );  
                    // }

                    //coda_send_seriale.push(HEADER_A);
                    //coda_send_seriale.push(HEADER_B);
                    //coda_send_seriale.push(PAYLOAD_POSE);
                    //encode_payload(new_grafic[54]);
                    //encode_payload(new_grafic[55]);
                    //encode_payload(new_grafic[56]);
                    

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
