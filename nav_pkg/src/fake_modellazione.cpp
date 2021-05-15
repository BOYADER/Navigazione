#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "nav_pkg/usbl.h"
#include "nav_pkg/ahrs.h"
#include "nav_pkg/imu.h"
#include "nav_pkg/dvl.h"
#include "nav_pkg/depth.h"

float time_step = 0.1;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_modellazione");
	ros::NodeHandle node_obj;
	
	ros::Publisher pub_imu =  node_obj.advertise<nav_pkg::imu>("/imu",10);
	ros::Publisher pub_ahrs =  node_obj.advertise<nav_pkg::ahrs>("/ahrs",10);
	ros::Publisher pub_usbl =  node_obj.advertise<nav_pkg::usbl>("/usbl",10);
	ros::Publisher pub_depth =  node_obj.advertise<nav_pkg::depth>("/depth",10);
	ros::Publisher pub_dvl =  node_obj.advertise<nav_pkg::dvl>("/dvl",10);
	ros::Rate loop_rate(1/time_step);	//10 Hz Sensor Frequency

	while(ros::ok())
	{
		nav_pkg::imu imu_msg;
		
		imu_msg.acc.x=1;
		imu_msg.acc.y=2;
		imu_msg.acc.z=3;
	
		imu_msg.gyro.x=1;
		imu_msg.gyro.y=2;
		imu_msg.gyro.z=3;

		imu_msg.counter =ros::Time::now().toSec();
		
		nav_pkg::ahrs ahrs_msg;
	
		ahrs_msg.rpy.x=1;	
		ahrs_msg.rpy.y=2;
		ahrs_msg.rpy.z=3;
		
		ahrs_msg.counter= ros::Time::now().toSec();

		nav_pkg::usbl usbl_msg;
		
		usbl_msg.pos.x=1;
		usbl_msg.pos.y=2;
		usbl_msg.pos.z=0;
		usbl_msg.counter=ros::Time::now().toSec();

		nav_pkg::depth depth_msg;

		depth_msg.z=10 * sin(ros::Time::now().toSec());
		depth_msg.counter=ros::Time::now().toSec();

		nav_pkg::dvl dvl_msg;
	
		dvl_msg.lin_vel.x=1;
		dvl_msg.lin_vel.y=2;
		dvl_msg.lin_vel.z=3;
		dvl_msg.counter=ros::Time::now().toSec();

		pub_imu.publish(imu_msg);
		pub_ahrs.publish(ahrs_msg);
		pub_usbl.publish(usbl_msg);
		pub_depth.publish(depth_msg);
		pub_dvl.publish(dvl_msg);

		loop_rate.sleep();
		ros::spinOnce();
	}

	

return 0;

}
