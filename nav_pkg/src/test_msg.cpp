#include "ros/ros.h"
#include "nav_pkg/Odom.h"
#include "nav_pkg/usbl.h"
#include "nav_pkg/ahrs.h"
#include "nav_pkg/imu.h"
#include "nav_pkg/dvl.h"
#include "nav_pkg/depth.h"

/*CALLBACK FUNCTIONS*/

//imu_callback
void imu_callback(const nav_pkg::imu::ConstPtr& msg)
{
	ROS_WARN("IMU DATA: \n");
	ROS_INFO("Accelerometro: [%f, %f, %f]\n", msg->acc_x, msg->acc_y, msg->acc_z);
	ROS_INFO("Giroscopio: [%f, %f, %f]\n", msg->gyro_x, msg->gyro_y, msg->gyro_z);
}

void ahrs_callback(const nav_pkg::ahrs::ConstPtr& msg)
{
	ROS_WARN("AHRS DATA: \n");
	ROS_INFO("Orientamento: [%f, %f, %f]\n", msg->roll, msg->pitch, msg->yaw);
}

void usbl_callback(const nav_pkg::usbl::ConstPtr& msg)
{
	ROS_WARN("USBL DATA: \n");
	ROS_INFO("Posizione: [%f, %f]\n", msg->x, msg->y);
}

void depth_callback(const nav_pkg::depth::ConstPtr& msg)
{
	ROS_WARN("DEPTH DATA: \n");
	ROS_INFO("Profondita: %f\n", msg->z);
}

void dvl_callback(const nav_pkg::dvl::ConstPtr& msg)
{
	ROS_WARN("DVL DATA: \n");
	ROS_INFO("Velocita: [%f, %f, %f]\n", msg->x, msg->y, msg->z);
}


/*MAIN*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_msg");
	ros::NodeHandle node_obj;
	
	ros::Publisher pub =  node_obj.advertise<nav_pkg::Odom>("/odom",10);
	ros::Subscriber sub_imu=node_obj.subscribe("/imu", 1000, imu_callback);
	ros::Subscriber sub_ahrs=node_obj.subscribe("/ahrs", 1000, ahrs_callback);
	ros::Subscriber sub_usbl=node_obj.subscribe("/usbl", 1000, usbl_callback);
	ros::Subscriber sub_dvl=node_obj.subscribe("/dvl", 1000, dvl_callback);
	ros::Subscriber sub_depth=node_obj.subscribe("/depth", 1000, depth_callback);
	ros::Rate loop_rate(10);	//10 Hz Prediction step

	while(ros::ok())
	{
		ros::spinOnce();
		nav_pkg::Odom odom_msg;
		
		odom_msg.lat=0;
		odom_msg.longit=0;
		odom_msg.h=0;
		odom_msg.roll=0;
		odom_msg.pitch=0;
		odom_msg.yaw=0;

		ROS_WARN("Stima: \n");
		ROS_INFO("Latitudine: %f\n",odom_msg.lat);
		ROS_INFO("Longitudine: %f\n",odom_msg.longit);
		ROS_INFO("Quota: %f\n",odom_msg.h);
		ROS_INFO("Roll:%f\n",odom_msg.roll);
		ROS_INFO("Pitch:%f\n",odom_msg.pitch);
		ROS_INFO("Yaw:%f\n",odom_msg.yaw);

		pub.publish(odom_msg);
		loop_rate.sleep();
	}

	

return 0;

}
