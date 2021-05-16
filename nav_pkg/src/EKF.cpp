#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "nav_pkg/Odom.h"
#include "nav_pkg/usbl.h"
#include "nav_pkg/ahrs.h"
#include "nav_pkg/imu.h"
#include "nav_pkg/dvl.h"
#include "nav_pkg/depth.h"
#include "nav_pkg/gps.h"
#include <nav_pkg/sensor_utility.h>
#include <nav_pkg/geolib.h>

using namespace std;
using namespace Eigen;

/*------------PARAMETERS DEFINITIONS------------*/
#define T 0.1
#define DEPTH 50 //profondità fondale
#define LAT_0 45.110735 //latitudine orig. {NED} 
#define LONG_0 7.640827 //longitudine orig. {NED}
#define ALT_0 0			//altitudine orig. {NED}
#define n 9 //num. state variable
#define p 7 //num. max y
#define N_init 10 //num variabili di init

Vector3f ned_lla_0(LAT_0, LONG_0, ALT_0);
Vector3f ned_ecef_0 = geodetic2Ecef(ned_lla_0);

//Non so perche non compila con il comma init fuori dal main
Vector3f eta1_0;
VectorXf x0(n); 
//x0 << 1, 2, 3, 4, 5, 6, 0.4, 0.2, 0.5; 
VectorXf P0(n);
//P0 << 1, 2, 3, 4, 5, 6, 0.2, 0.4, 0.5;
VectorXf process_noise(n);
//process_noise << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;
VectorXf sensor_noise(p);
//sensor_noise << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;

/*------------CUSTOM FUNCTION DECLARATION------------*/
//eigen-ros interface
geometry_msgs::Vector3 eigen2ros(Vector3f v);
Vector3f ros2eigen(geometry_msgs::Vector3 v_msg);
void print_info(VectorXf state_corr, Vector3f lla, Vector3f rpy, MatrixXf P_corr);

/*---------GLOBAL VARIABLES----------*/
//input: acc - gyro - orientam.
geometry_msgs::Vector3 acceler;
float acceler_id;
geometry_msgs::Vector3 gyrosc;
float gyrosc_id;
geometry_msgs::Vector3 ahrs;
float ahrs_id;

//sens. non inerziali
DEPTH_sensor depth_obj;
USBL_sensor usbl_obj;
DVL_sensor dvl_obj;
GPS_sensor gps_obj;

float prev_id = 0;

/*------------CALLBACK FUNCTIONS------------*/
void imu_callback(const nav_pkg::imu::ConstPtr& msg)
{
	acceler = msg->acc;
	gyrosc = msg->gyro;
	acceler_id = msg->counter;
	gyrosc_id = msg->counter;
	/*ROS_WARN("IMU DATA: \n");
	ROS_INFO("Accelerometro: [%f, %f, %f]\n", msg->acc.x, msg->acc.y, msg->acc.z);
	ROS_INFO("Giroscopio: [%f, %f, %f]\n", msg->gyro.x, msg->gyro.y, msg->gyro.z);*/
}

void ahrs_callback(const nav_pkg::ahrs::ConstPtr& msg)
{
	ahrs = msg->rpy;
	ahrs_id = msg->counter;
	/*ROS_WARN("AHRS DATA: \n");
	ROS_INFO("Orientamento: [%f, %f, %f]\n", msg->rpy.x, msg->rpy.y, msg->rpy.z);*/
}

void usbl_callback(const nav_pkg::usbl::ConstPtr& msg)
{
	usbl_obj.data = msg->pos;
	usbl_obj.id = msg->counter;
	/*ROS_WARN("USBL DATA: \n");
	ROS_INFO("Posizione: [%f, %f, %f]\n", msg->pos.x, msg->pos.y, msg->pos.z);*/
}

void depth_callback(const nav_pkg::depth::ConstPtr& msg)
{
	depth_obj.data.data = msg->z;
	depth_obj.id= msg->counter;
	/*ROS_WARN("DEPTH DATA: \n");
	ROS_INFO("Profondita: %f\n", msg->z);*/
}

void dvl_callback(const nav_pkg::dvl::ConstPtr& msg)
{
	dvl_obj.data = msg->lin_vel;
	dvl_obj.id = msg->counter;
	/*ROS_WARN("DVL DATA: \n");
	ROS_INFO("Velocita: [%f, %f, %f]\n", msg->lin_vel.x, msg->lin_vel.y, msg->lin_vel.z);*/
}

void gps_callback(const nav_pkg::gps::ConstPtr& msg)
{
	gps_obj.data = msg->lla;
	gps_obj.id = msg->counter;
	gps_obj.under_water = msg->under_water;
	/*ROS_WARN("GPS DATA: \n");
	ROS_INFO("LLA: [Lat: %f, Long: %f, Alt: %f]\n", msg->lla.x, msg->lla.y, msg->lla.z);
	ROS_INFO("Counter: %f", msg->counter);*/

}

/*------------MAIN------------*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "EKF");
	ros::NodeHandle node_obj;
	
	ros::Publisher pub =  node_obj.advertise<nav_pkg::Odom>("/odom",10);
	ros::Subscriber sub_imu=node_obj.subscribe("/imu", 1000, imu_callback);
	ros::Subscriber sub_ahrs=node_obj.subscribe("/ahrs", 1000, ahrs_callback);
	ros::Subscriber sub_usbl=node_obj.subscribe("/usbl", 1000, usbl_callback);
	ros::Subscriber sub_dvl=node_obj.subscribe("/dvl", 1000, dvl_callback);
	ros::Subscriber sub_depth=node_obj.subscribe("/depth", 1000, depth_callback);
	ros::Subscriber sub_gps=node_obj.subscribe("/gps", 1000, gps_callback);
	ros::Rate loop_rate(1/T);	//10 Hz Prediction step

	//init useful tool
	static int isInit = 0;
	int streak = 0;

	ROS_INFO("Initialization\n");
	while (isInit == 0) 
	{
		loop_rate.sleep();
		ros::spinOnce();

		if (( gps_obj.id != prev_id )&&(!gps_obj.under_water))
		{
			++streak;
		}

		else if (( gps_obj.id!=prev_id )&&(gps_obj.under_water))
		{
			streak = 0;
		}

		ROS_INFO("n. Misure: %d, GPS_ID: %f, Prev_ID: %f\n", streak, gps_obj.id, prev_id);

		if ( streak>=N_init )
		{
			eta1_0 << geodetic2Ned(ros2eigen(gps_obj.data), ned_lla_0);
			eta1_0(2) = depth_obj.data.data;
			isInit = 1;
			ROS_INFO("eta1_0 {NED}: [%f, %f, %f]\n", eta1_0(0), eta1_0(1), eta1_0(2));
			ROS_INFO("Inizializzazione completata. Boia Deh!\n");
		}

		prev_id = gps_obj.id;

	}

	//Purtroppo devo caricarli dentro il main
	x0 << eta1_0(0), eta1_0(1), eta1_0(2), 0, 0, 0, 0, 0, 0; 
	P0 << 1, 2, 3, 4, 5, 6, 0.2, 0.4, 0.5;
	process_noise << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;
	sensor_noise << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
	
	//variabili di stato
	Vector3f eta1(x0(0), x0(1), x0(2)); //pos {NED}
	Vector3f ni1(x0(3), x0(4), x0(5)); //linear vel {BODY}
	Vector3f ni2(x0(6), x0(7), x0(8)); //angular vel. {BODY}

	MatrixXf P = P0.asDiagonal(); //Covariance
	MatrixXf Q = process_noise.asDiagonal(); //Process Noise Cov.
	MatrixXf R = sensor_noise.asDiagonal(); //Sensor Noise Cov.

	while(ros::ok())
	{
		ros::spinOnce();	//ricevo i dati dai sensori

		//Rotation matrix
		Matrix3f J1= Jacobian_RPY(ros2eigen(ahrs));

		/*PREDICTION --> process noise = 0 */
		eta1 = eta1 + T*J1*ni1;
		ni1= ni1;
		ni2= ni2; 

		VectorXf state_pred(9);
		state_pred << eta1, ni1, ni2;

		/*ROS_WARN("Stato Predetto\n");
		cout << state_pred << endl;*/

		MatrixXf F = vileMatriceF(ros2eigen(ahrs));
		//cout << F << endl;
	
		MatrixXf D = vileMatriceD(ni1, ros2eigen(ahrs));
		//cout << D << endl;

		//Aggiorn. della cov.
		P = F*P*F.transpose() + D*Q*D.transpose();
		//cout << P << endl;

		/*CORRECTION --> sensor noise = 0*/
		VectorXf sensor_vector(p); //y_k
		sensor_vector << ros2eigen(usbl_obj.data), ros2eigen(ahrs), depth_obj.data.data;

		VectorXf sensor_function(p); //h(x_pred, 0)
		sensor_function << usbl_obj.usbl_function(eta1), dvl_obj.dvl_function(ni1, ni2), depth_obj.depth_function(eta1);

		VectorXf e(p);
		e = sensor_vector - sensor_function;

		//correction matrix
		MatrixXf H = vileMatriceH(eta1, ni1, ni2);
		//cout << H << endl;

		MatrixXf M = MatrixXf::Identity(7, 7);
		//cout << M << endl;

		//Correction Algorithm
		MatrixXf S = H*P*H.transpose() + M*R*M.transpose(); //7x7
		MatrixXf L = P*H.transpose()*S.inverse(); //9x7

		VectorXf state_corr = state_pred + L*e;
		MatrixXf useful_term(9, 9); 
		useful_term = MatrixXf::Identity(9, 9) - L*H; //9x9
		P = useful_term*P*useful_term.transpose() + L*R*L.transpose();

		/*ROS_WARN("Stato Corretto\n");
		cout << state_corr << endl;
		cout << P << endl;*/

		eta1 << state_corr(0), state_corr(1), state_corr(2);
		ni1 << state_corr(3), state_corr(4), state_corr(5);
		ni2 << state_corr(6), state_corr(7), state_corr(8);

		/*LLA conversion*/
		//ROS_WARN("TEST LLA\n");
		//cout << ned_ecef_0 << endl;
		Vector3f lla_test = ned2Geodetic(eta1, ned_ecef_0); //integrare in un'unica funz.
		//cout << lla_test << endl;
		
		//print_info(state_corr, lla_test, ros2eigen(ahrs), P);

		//Publish
		nav_pkg::Odom odom_msg;
		
		odom_msg.lla=eigen2ros(lla_test);
		odom_msg.lin_vel=eigen2ros(ni1);
		odom_msg.rpy=ahrs;

		pub.publish(odom_msg);
		loop_rate.sleep();
	}

return 0;
}

void print_info(VectorXf state_corr, Vector3f lla, Vector3f rpy, MatrixXf P_corr)
{
	ROS_WARN("|********EKF RESULT*********|\n");

	ROS_INFO("Posizione: {NED} \n");
	ROS_INFO("x:%f y:%f z:%f\n", state_corr(0), state_corr(1), state_corr(2));
	ROS_INFO("Lat. %f, Long. %f, Altit. %f\n", lla(0), lla(1), lla(2));

	ROS_INFO("Velocita' Lineare {BODY}\n");
	ROS_INFO("u:%f v:%f w:%f\n", state_corr(3), state_corr(4), state_corr(5));

	ROS_INFO("RPY ANGLES:\n");
	ROS_INFO("roll:%f pitch:%f yaw:%f\n", rpy(0), rpy(1), rpy(2));

	ROS_INFO("Velocita' Angolare {BODY}\n");
	ROS_INFO("p:%f q:%f r:%f\n", state_corr(6), state_corr(7), state_corr(8));

	ROS_INFO("Covarianza: \n");
	cout << P_corr << endl;
}

geometry_msgs::Vector3 eigen2ros(Vector3f v)
{
	geometry_msgs::Vector3 v_msg;
	v_msg.x=v(0);
	v_msg.y=v(1);
	v_msg.z=v(2);
	return v_msg;
}

Vector3f ros2eigen(geometry_msgs::Vector3 v_msg)
{
	Vector3f v(v_msg.x, v_msg.y, v_msg.z);
	return v;
}
