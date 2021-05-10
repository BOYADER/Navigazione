#include "ros/ros.h"
#include "nav_pkg/usbl.h"
#include "nav_pkg/ahrs.h"
#include "nav_pkg/imu.h"
#include "nav_pkg/dvl.h"
#include "nav_pkg/depth.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_modellazione");
	ros::NodeHandle node_obj;
	
	ros::Publisher pub_imu =  node_obj.advertise<nav_pkg::imu>("/imu",10);
	ros::Publisher pub_ahrs =  node_obj.advertise<nav_pkg::ahrs>("/ahrs",10);
	ros::Publisher pub_usbl =  node_obj.advertise<nav_pkg::usbl>("/usbl",10);
	ros::Publisher pub_depth =  node_obj.advertise<nav_pkg::depth>("/depth",10);
	ros::Publisher pub_dvl =  node_obj.advertise<nav_pkg::dvl>("/dvl",10);
	ros::Rate loop_rate(10);	//10 Hz Sensor Frequency

	while(ros::ok())
	{
		nav_pkg::imu imu_msg;
		
		imu_msg.acc_x=0;
		imu_msg.acc_y=0;
		imu_msg.acc_z=0;
	
		imu_msg.gyro_x=0;
		imu_msg.gyro_y=0;
		imu_msg.gyro_z=0;
		
		nav_pkg::ahrs ahrs_msg;
	
		ahrs_msg.roll=0;	
		ahrs_msg.pitch=0;
		ahrs_msg.yaw=0;

		nav_pkg::usbl usbl_msg;
		
		usbl_msg.x=0;
		usbl_msg.y=0;

		nav_pkg::depth depth_msg;

		depth_msg.z=10 * sin(ros::Time::now().toSec());

		nav_pkg::dvl dvl_msg;
	
		dvl_msg.x=0;
		dvl_msg.y=0;
		dvl_msg.z=0;

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
