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

/*------------PARAMETERS DEFINITIONS------------*/
//dimension State Space
#define n 12
#define N 10

//Init Waypoint
float LAT_0;	//[deg]
float LONG_0;	//[deg]
float ALT_0;	//[m]

int N_init;		//n° misure per init

//Deviaz. Standard dei Sensori
float dev_gps;	//[m]
float dev_r;	//[m]
float dev_be;	//[deg]
float dev_rp;	//[deg]
float dev_y;	//[deg]
float dev_z;	//[m]
float dev_gyro;	//[rad/s]

/*------------CUSTOM FUNCTION DECLARATION------------*/
void print_info(VectorXf state_corr, Vector3f lla, MatrixXf P_corr);
void param_Init(ros::NodeHandle node_obj);

/*SENSOR OBJECTS*/
Sensor gyro_obj; //nota: spiegare perche' Sensor e non Gyro::Sensor
Depth depth_obj;
Usbl usbl_obj;
Dvl dvl_obj;
Gps gps_obj;
Ahrs ahrs_obj;

/*------------CALLBACK FUNCTIONS------------*/

void ahrs_callback(const nav_pkg::ahrs::ConstPtr& msg)
{
	//ahrs
	ahrs_obj.set_data(msg->rpy);
	ahrs_obj.set_id(msg->counter);
	ahrs_obj.set_isNew();

	//gyro
	gyro_obj.set_data(msg->gyro);
	gyro_obj.set_id(msg->counter);
	gyro_obj.set_isNew();
	/*ROS_WARN("AHRS DATA: \n");
	ROS_INFO("Orientamento: [%f, %f, %f]\n", msg->rpy.x, msg->rpy.y, msg->rpy.z);
	ROS_INFO("Angular Rate: [%f, %f, %f]\n", msg->gyro.x, msg->gyro.y, msg->gyro.z);*/
}

void usbl_callback(const nav_pkg::usbl::ConstPtr& msg)
{
	usbl_obj.set_data(msg->pos);
	usbl_obj.set_id(msg->counter);
	usbl_obj.set_isNew();
	/*ROS_WARN("USBL DATA: \n");
	ROS_INFO("Posizione: [%f, %f, %f]\n", msg->pos.x, msg->pos.y, msg->pos.z);*/
}

void depth_callback(const nav_pkg::depth::ConstPtr& msg)
{
	depth_obj.set_data(msg->z);
	depth_obj.set_id(msg->counter);
	depth_obj.set_isNew();
	/*ROS_WARN("DEPTH DATA: \n");
	ROS_INFO("Profondita: %f\n", msg->z);*/
}

void dvl_callback(const nav_pkg::dvl::ConstPtr& msg)
{
	dvl_obj.set_data(msg->lin_vel);
	dvl_obj.set_id(msg->counter);
	dvl_obj.set_isNew();
	/*ROS_WARN("DVL DATA: \n");
	ROS_INFO("Velocita: [%f, %f, %f]\n", msg->lin_vel.x, msg->lin_vel.y, msg->lin_vel.z);*/
}

void gps_callback(const nav_pkg::gps::ConstPtr& msg)
{
	gps_obj.set_data(msg->lla);
	gps_obj.set_id(msg->counter);
	gps_obj.set_isNew();
	gps_obj.set_underWater(msg->under_water);
	/*ROS_WARN("GPS DATA: \n");
	ROS_INFO("LLA: [Lat: %f, Long: %f, Alt: %f]\n", msg->lla.x, msg->lla.y, msg->lla.z);
	ROS_INFO("Counter: %f", msg->counter);*/
}

/*------------MAIN------------*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "PF");
	ros::NodeHandle node_obj;
	
	ros::Publisher pub =  node_obj.advertise<nav_pkg::Odom>("/odom",10);
	ros::Subscriber sub_ahrs=node_obj.subscribe("/ahrs", 1, ahrs_callback);
	ros::Subscriber sub_usbl=node_obj.subscribe("/usbl", 1, usbl_callback);
	ros::Subscriber sub_dvl=node_obj.subscribe("/dvl", 1, dvl_callback);
	ros::Subscriber sub_depth=node_obj.subscribe("/depth", 1, depth_callback);
	ros::Subscriber sub_gps=node_obj.subscribe("/gps", 1, gps_callback);
	ros::Rate loop_rate(1/T);	//10 Hz Prediction step

	//Carico i parametri da mission.yaml
	param_Init(node_obj);

	Vector3f ned_lla_0(LAT_0, LONG_0, ALT_0);

	//init useful tool
	static int isInit = 0;
	int streak = 0;

	Vector3f eta1;  
	Vector3f ni1;
	Vector3f eta2;
    Vector3f ni2;

	ROS_WARN("Initialization");
	while (isInit == 0) 
	{
		loop_rate.sleep();
		ros::spinOnce();

		ROS_INFO("Init: Id: %f, IsNew: %d", gps_obj.get_id(), gps_obj.get_isNew());

		bool valid_meas = gps_obj.get_isNew() && !gps_obj.get_underWater() && ahrs_obj.get_isNew() && dvl_obj.get_isNew() && depth_obj.get_isNew();

		if (valid_meas)
			++streak;

		else if ((gps_obj.get_isNew())&&(gps_obj.get_underWater()))
			streak = 0;

		if (streak >= N_init)
		{
			eta1 << lla2ned(ros2eigen(gps_obj.get_data()), ned_lla_0);
			eta1(2) = depth_obj.get_data();
			ni1 << ros2eigen(dvl_obj.get_data());
			eta2 << ros2eigen(ahrs_obj.get_data());
            ni2 << ros2eigen(gyro_obj.get_data());

			isInit = 1;
			ROS_INFO("eta1_0 {NED}: [%f, %f, %f]\n", eta1(0), eta1(1), eta1(2));
			ROS_INFO("ni1_0 {BODY}: [%f, %f, %f]\n", ni1(0), ni1(1), ni1(2));
			ROS_INFO("eta2_0 {NED}: [%f, %f, %f]\n", eta2(0), eta2(1), eta2(2));
            ROS_INFO("ni2_0 {BODY}: [%f, %f, %f]\n", ni2(0), ni2(1), ni2(2));
			ROS_INFO("Inizializzazione completata. Boya Dehr!\n");
		}

		gps_obj.set_isOld();
		dvl_obj.set_isOld();
		depth_obj.set_isOld();
		ahrs_obj.set_isOld();
	}

	VectorXf process_noise(n);
	VectorXf sensor_noise(13);
	VectorXf P0(n);

	P0 << pow(dev_gps, 2), pow(dev_gps, 2), pow(dev_z, 2), 0.4, 0.5, 0.6, pow(deg2Rad(dev_rp),2),  pow(deg2Rad(dev_rp),2), pow(deg2Rad(dev_y), 2), 0.1, 0.1, 0.1; 
	process_noise << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.1, 0.1, 0.1; //da definire	

	MatrixXf P = P0.asDiagonal(); //Covariance
	MatrixXf Q = process_noise.asDiagonal(); //Process Noise Cov.

    //Generazione particelle sullo stato iniziale
    VectorXf x1_0(n);
    x1_0 << eta1, ni1, eta2, ni2; //12

    EigenMultivariateNormal<float> normX_solver(x1_0, P);
    MatrixXf particles = normX_solver.samples(N);
    VectorXf pesi(N);
    pesi.setOnes();

    ROS_INFO("Pesi\n");
    cout << pesi << endl;

   /*pesi *= (1/N);

    ROS_INFO("Pesi Normalizzati\n");
    cout << pesi << endl;*/

	while(ros::ok())
	{
		ros::spinOnce();	//ricevo i dati dai sensori		
        
        //Rotation matrix
		Matrix3f J1= Jacobian_RPY(eta2);
		

		//Publish
		/*nav_pkg::Odom odom_msg;
		
		odom_msg.lld=eigen2ros(lld);
		odom_msg.lin_vel=eigen2ros(ni1);
		odom_msg.rpy=eigen2ros(eta2);

		pub.publish(odom_msg);*/
		loop_rate.sleep();
	}

return 0;
}

void print_info(VectorXf state_corr, Vector3f lla, MatrixXf P_corr)
{
	ROS_WARN("|********EKF RESULT*********|\n");

	ROS_INFO("Posizione: {NED} \n");
	ROS_INFO("x:%f y:%f z:%f\n", state_corr(0), state_corr(1), state_corr(2));
	ROS_INFO("Lat. %f, Long. %f, Altit. %f\n", lla(0), lla(1), lla(2));

	ROS_INFO("Velocita' Lineare {BODY}\n");
	ROS_INFO("u:%f v:%f w:%f\n", state_corr(3), state_corr(4), state_corr(5));

	ROS_INFO("RPY ANGLES:\n");
	ROS_INFO("roll:%f pitch:%f yaw:%f\n", state_corr(6), state_corr(7), state_corr(8));

	ROS_INFO("Covarianza: \n");
	cout << clear_small_number(P_corr) << endl;
}

void param_Init(ros::NodeHandle node_obj)
{
	//Carico i parametri da mission.yaml
	node_obj.getParam("/initial_pose/position/latitude", LAT_0);
	node_obj.getParam("/initial_pose/position/longitude", LONG_0);
	node_obj.getParam("/initial_pose/position/depth", ALT_0);

	node_obj.getParam("/N_init", N_init);

	node_obj.getParam("/sensor_dev/gps", dev_gps);
	node_obj.getParam("/sensor_dev/usbl/range", dev_r);
	node_obj.getParam("/sensor_dev/usbl/b_e", dev_be);
	node_obj.getParam("/sensor_dev/depth", dev_z);
	node_obj.getParam("/sensor_dev/ahrs/r_p", dev_rp);
	node_obj.getParam("/sensor_dev/ahrs/yaw", dev_y);
	node_obj.getParam("/sensor_dev/gyro", dev_gyro);
}