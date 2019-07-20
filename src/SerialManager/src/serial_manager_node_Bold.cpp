#include "serial_manager.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Serial_Manager");
  ros::NodeHandle n;
  
ros::Subscriber forward_pose_topic = n.subscribe<geometry_msgs::Point>("tag_pose0", 1, &Pose_cb_forward);
ros::Subscriber backward_pose_topic = n.subscribe<geometry_msgs::Point>("tag_pose1", 1, &Pose_cb_back);
ros::Subscriber waypoint = n.subscribe<geometry_msgs::Point>("/waypoint_publisher", 1, &waypoint_cb);
ros::Subscriber start_and_stop=n.subscribe<std_msgs::Float64>("/start_and_stop", 1, &start_and_stop_cb);
  
  n.param<std::string>("/SerialManager/dev", seriale_dev, "/dev/ttyS0");
  n.param<bool>("/SerialManager/stream_pose", stream_pose, true);
  n.param<int>("/SerialManager/pose_el_time", pose_el_time, 500);

  seriale_dev= "/dev/ttyS0";
  int result = serial_init(&serial, seriale_dev.c_str());
   way_x=2;
   way_y=2;
   way_z=2;
   startstop=2;
   pos_x_f=2;
   pos_y_f=2;
   pos_z_f=2;
   pos_x_b=2;
   pos_y_b=2;
   pos_z_b=2;
   
   
  gettimeofday(&stream_pose_time, NULL);
  ros::Rate loop_rate(100);
  if (result == 1)
  {
      while(ros::ok())
      {
      stream_pose=true;
      new_packet_pose=true;

      
        if(stream_pose && new_packet_pose)
        {
          ROS_INFO("dato ricevuto");
             pos_x_f=104;
        pos_y_f=105;
        pos_z_f=103;
         pos_x_b=100;
        pos_y_b=240;
        pos_z_b=240;
        way_x = 7;
        way_y = 56;
        way_z = 150;
        startstop = 30;
          //cout<<"elapsed time:"<<elapsed_time_pose<<endl;
          //devo inviare la posizione letta
          coda_send_seriale.push(HEADER_A);
          coda_send_seriale.push(HEADER_B);
          coda_send_seriale.push(PAYLOAD_POSE);
          encode_payload(pos_x_f);
          encode_payload(pos_y_f);
          encode_payload(pos_z_f);
          encode_payload(pos_x_b);
          encode_payload(pos_y_b);
          encode_payload(pos_z_b);
     //     encode_payload(way_x);
     //     encode_payload(way_y);
     //     encode_payload(way_z );
     //     encode_payload(startstop);

          new_packet_pose = 0;

        write_to_serial(&serial);
        }
        ros::spinOnce();
        loop_rate.sleep();
      }
  }
  return 0;
}
