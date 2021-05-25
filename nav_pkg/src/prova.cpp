#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "nav_pkg/Odom.h"
#include "nav_pkg/usbl.h"
#include "nav_pkg/ahrs.h"
#include "nav_pkg/dvl.h"
#include "nav_pkg/depth.h"
#include "nav_pkg/gps.h"
#include <nav_pkg/sensor_class.h>
#include <nav_pkg/geolib.h>

#include <nav_pkg/eigenmvn.h>

using namespace std;
using namespace Eigen;

/*---------GLOBAL VARIABLES----------*/
//sens. non inerziali
Depth depth_obj;
Usbl usbl_obj;
Dvl dvl_obj;
Ahrs ahrs_obj;

int N = 1000;

/*------------CALLBACK FUNCTIONS------------*/

void ahrs_callback(const nav_pkg::ahrs::ConstPtr& msg)
{
	ahrs_obj.set_data(msg->rpy);
	ahrs_obj.set_id(msg->counter);
	ahrs_obj.set_isNew();
	//ROS_INFO("Orientamento: [%f, %f, %f]\n", msg->rpy.x, msg->rpy.y, msg->rpy.z);
}

void usbl_callback(const nav_pkg::usbl::ConstPtr& msg)
{
	usbl_obj.set_data(msg->pos);
	usbl_obj.set_id(msg->counter);
	usbl_obj.set_isNew();
	//ROS_INFO("Posizione: [%f, %f, %f]\n", msg->pos.x, msg->pos.y, msg->pos.z);
}

void depth_callback(const nav_pkg::depth::ConstPtr& msg)
{
	depth_obj.set_data(msg->z);
	depth_obj.set_id(msg->counter);
	depth_obj.set_isNew();
	//ROS_INFO("Profondita: %f\n", msg->z);
}

void dvl_callback(const nav_pkg::dvl::ConstPtr& msg)
{
	dvl_obj.set_data(msg->lin_vel);
	dvl_obj.set_id(msg->counter);
	dvl_obj.set_isNew();
	//ROS_INFO("Velocita: [%f, %f, %f]\n", msg->lin_vel.x, msg->lin_vel.y, msg->lin_vel.z);
}

/*------------MAIN------------*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "prova");
	ros::NodeHandle node_obj;
	
	ros::Publisher pub =  node_obj.advertise<nav_pkg::Odom>("/odom",10);
	ros::Subscriber sub_ahrs=node_obj.subscribe("/ahrs", 1, ahrs_callback);
	ros::Subscriber sub_usbl=node_obj.subscribe("/usbl", 1, usbl_callback);
	ros::Subscriber sub_dvl=node_obj.subscribe("/dvl", 1, dvl_callback);
	ros::Subscriber sub_depth=node_obj.subscribe("/depth", 1, depth_callback);
	ros::Rate loop_rate(1/T);	//10 Hz Prediction step

	while(ros::ok())
	{
		ros::spinOnce();	//ricevo i dati dai sensori
		loop_rate.sleep();

       	ROS_WARN("Test\n");

		VectorXf mean = VectorXf::Zero(9); 
		EigenMultivariateNormal<float> normX_solver(mean, MatrixXf::Identity(9, 9));

		MatrixXf test = normX_solver.samples(N); //9xN

		ahrs_obj.set_isOld();
		usbl_obj.set_isOld();
		dvl_obj.set_isOld();
		depth_obj.set_isOld();
	}

return 0;
}