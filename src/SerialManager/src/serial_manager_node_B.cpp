#include "serial_manager.h"

int main(int argc, char **argv)
{

ros::init(argc, argv, "Serial_Manager");
ros::NodeHandle n;
/******************INSTANZIATE ALL THE SUBSCRIBERS AND PUBLISHER***********************************************************************/

ros::Subscriber forward_pose_topic = n.subscribe<geometry_msgs::Point>("/turtle1/tag_pose0", 1, &Pose_cb_forward);
ros::Subscriber backward_pose_topic = n.subscribe<geometry_msgs::Point>("/turtle1/tag_pose1", 1, &Pose_cb_back);
ros::Subscriber waypoint = n.subscribe<geometry_msgs::Point>("/turtle1/waypoint_publisher", 1, &waypoint_cb);
ros::Subscriber start_and_stop=n.subscribe<std_msgs::Float64>("/turtle1/start_and_stop", 1, &start_and_stop_cb);
ros::Publisher heading_angle= n.advertise<geometry_msgs::Point>("/turtle1/orientation",100);
/******************DEFAULT PARAMETERS FOR THE SERIAL PORT***********************************************************************/

  n.param<std::string>("/SerialManager/dev", seriale_dev, "/dev/ttyS0");
  n.param<bool>("/SerialManager/stream_pose", stream_pose, true);
  n.param<int>("/SerialManager/pose_el_time", pose_el_time, 500);
  seriale_dev= "/dev/ttyS0";

  int result = serial_init(&serial, seriale_dev.c_str());

    
  gettimeofday(&stream_pose_time, NULL);
  ros::Rate loop_rate(100);
  if (result == 1)
  {
      while(ros::ok())
      {

          /******************************************************************************************/
          /******************READ MESSAGE FROM SERIAL ************************************************/
          /******************************************************************************************/
          read_from_serial(&serial);
          h_angle.z=orient_;
          heading_angle.publish(h_angle);

          /******************************************************************************************/
          /******************WRITE MESSAGE TO SERIAL ************************************************/
          /******************The following structure is required for ********************************/
          /******************the state machine on STM which is equivalent ***************************/
          /******************to the state machine implemented in this code **************************/
          /******************************************************************************************/
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
        
        
        }
        ros::spinOnce();
        loop_rate.sleep();
      
  }
  return 0;
}
