#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "nav_pkg/usbl.h"
#include "nav_pkg/ahrs.h"
#include "nav_pkg/dvl.h"
#include "nav_pkg/depth.h"
#include "nav_pkg/gps.h"

#include <nav_pkg/geolib.h>

float time_step = 0.1;
int i=0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_modellazione");
	ros::NodeHandle node_obj;
	
	ros::Publisher pub_ahrs =  node_obj.advertise<nav_pkg::ahrs>("/sensor/ahrs",10);
	ros::Publisher pub_usbl =  node_obj.advertise<nav_pkg::usbl>("/sensor/usbl",10);
	ros::Publisher pub_depth =  node_obj.advertise<nav_pkg::depth>("/sensor/depth",10);
	ros::Publisher pub_dvl =  node_obj.advertise<nav_pkg::dvl>("/sensor/dvl",10);
	ros::Publisher pub_gps = node_obj.advertise<nav_pkg::gps>("/sensor/gps",10);
	ros::Rate loop_rate(1/time_step);	//10 Hz Sensor Frequency

	while(ros::ok())
	{
		i++;
		nav_pkg::ahrs ahrs_msg;
		
		ahrs_msg.acc.x=0;
		ahrs_msg.acc.y=0;
		ahrs_msg.acc.z=0;
	
		ahrs_msg.gyro.x=0;
		ahrs_msg.gyro.y=0;
		ahrs_msg.gyro.z=0;
		
		ahrs_msg.rpy.x=1;	
		ahrs_msg.rpy.y=0.3;
		ahrs_msg.rpy.z=0.5;
		
		ahrs_msg.counter= ros::Time::now().toSec();

		nav_pkg::usbl usbl_msg;

		usbl_msg.pos.x= 50; //range
		usbl_msg.pos.y= 1.92308; //bearing
		usbl_msg.pos.z= M_PI_2; //elevation
		usbl_msg.counter=ros::Time::now().toSec();

		nav_pkg::depth depth_msg;

		depth_msg.z=0; //10 * sin(ros::Time::now().toSec());
		depth_msg.counter=ros::Time::now().toSec();

		nav_pkg::dvl dvl_msg;
	
		dvl_msg.lin_vel.x=0;
		dvl_msg.lin_vel.y=0;
		dvl_msg.lin_vel.z=0;
		dvl_msg.counter=ros::Time::now().toSec();

		nav_pkg::gps gps_msg;

		gps_msg.lla.x=45.110735; //[DEG]
		gps_msg.lla.y=7.640827;  //[DEG]
		gps_msg.lla.z=0; //[m]
		gps_msg.counter= (float) i; //ros::Time::now().toSec();
		
		if(gps_msg.counter > 60)
		{
			gps_msg.under_water = 1;
			//ROS_WARN("Sott'acqua!\n");
		}
		else
			gps_msg.under_water = 0;

		pub_ahrs.publish(ahrs_msg);
		pub_usbl.publish(usbl_msg);
	
		if(i%2 == 0)
			pub_depth.publish(depth_msg);

		pub_dvl.publish(dvl_msg);
		pub_gps.publish(gps_msg);

		loop_rate.sleep();
		ros::spinOnce();
	}

	

return 0;

}