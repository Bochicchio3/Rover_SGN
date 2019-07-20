#include "serial_manager.h"

/*****************************************************************/
/*                                                               */
/*                 MAIN                                          */
/*****************************************************************/
int main(int argc, char **argv)
{
 
 
    geometry_msgs::Point h_angle
     
                //READ FROM FUCKING SERIAL

            
                 
 
 
    ros::init(argc, argv, "Serial_Manager");
    ros::NodeHandle n;   
	ros::Subscriber pose_cam = n.subscribe<geometry_msgs::PoseStamped>("/poseCam", 1, &Pose_cb);
    ros::Publisher heading_angle= n.advertise<geometry_msgs::Point>("/orientation",100);
    
    //leggo i parametri specificati nel launch file
    std::string seriale_dev;
    n.param<std::string>("/SerialManager/dev", seriale_dev, "/dev/ttyS1");


    //int serial;
    // init della seriale
    int result = serial_init(&serial, seriale_dev.c_str());
    if(result < 0 ) return 0;
 
    ros::Rate loop_rate(100); // 100 Hz 
    while(ros::ok()){
            read_from_serial(&serial);S
            h_angle.z=orient_;
            heading_angle.publish(h_angle);
            ros::spinOnce();
            loop_rate.sleep();
    }
   
  return 0;
}
