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

using namespace std;
using namespace Eigen;

/*------------PARAMETERS DEFINITIONS------------*/
//#define T 0.1
#define LAT_0 45.110735 //latitudine orig. {NED} 
#define LONG_0 7.640827 //longitudine orig. {NED}
#define ALT_0 0			//altitudine orig. {NED}
#define n 9 //num. state variable
#define N_init 10 //num variabili di init

Vector3f ned_lla_0(LAT_0, LONG_0, ALT_0);

/*------------CUSTOM FUNCTION DECLARATION------------*/
void print_info(VectorXf state_corr, Vector3f lla, Vector3f rpy, MatrixXf P_corr);

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
	ros::init(argc, argv, "EKF");
	ros::NodeHandle node_obj;
	
	ros::Publisher pub =  node_obj.advertise<nav_pkg::Odom>("/odom",10);
	ros::Subscriber sub_ahrs=node_obj.subscribe("/ahrs", 1, ahrs_callback);
	ros::Subscriber sub_usbl=node_obj.subscribe("/usbl", 1, usbl_callback);
	ros::Subscriber sub_dvl=node_obj.subscribe("/dvl", 1, dvl_callback);
	ros::Subscriber sub_depth=node_obj.subscribe("/depth", 1, depth_callback);
	ros::Subscriber sub_gps=node_obj.subscribe("/gps", 1, gps_callback);
	ros::Rate loop_rate(1/T);	//10 Hz Prediction step

	//init useful tool
	static int isInit = 0;
	int streak = 0;

	Vector3f eta1;  
	Vector3f ni1;
	Vector3f eta2;


	ROS_INFO("Initialization\n");
	while (isInit == 0) 
	{
		loop_rate.sleep();
		ros::spinOnce();

		ROS_WARN("Test Init: Id: %f, IsNew?: %d", gps_obj.get_id(), gps_obj.get_isNew());

		if (( gps_obj.get_isNew())&&(!gps_obj.get_underWater()))
		{
			++streak;
		}

		else if ((gps_obj.get_isNew())&&(gps_obj.get_underWater()))
		{
			streak = 0;
		}

		//ROS_INFO("n. Misure: %d\n", streak);

		if ( streak>=N_init )
		{
			eta1 << lla2ned(ros2eigen(gps_obj.get_data()), ned_lla_0);
			eta1(2) = depth_obj.get_data();
			ni1 << ros2eigen(dvl_obj.get_data());
			eta2 << ros2eigen(ahrs_obj.get_data());

			isInit = 1;
			ROS_INFO("eta1_0 {NED}: [%f, %f, %f]\n", eta1(0), eta1(1), eta1(2));
			ROS_INFO("ni1_0 {BODY}: [%f, %f, %f]\n", ni1(0), ni1(1), ni1(2));
			ROS_INFO("eta2_0 {NED}: [%f, %f, %f]\n", eta2(0), eta2(1), eta2(2));
			ROS_INFO("Inizializzazione completata. Boya Dehr!\n");
		}

		gps_obj.set_isOld();
	}

	VectorXf process_noise(n);
	VectorXf sensor_noise(13);
	VectorXf P0(n);

	P0 << 1, 1, 1, 0.4, 0.5, 0.6, pow(deg2Rad(0.03),2),  pow(deg2Rad(0.03),2), deg2Rad(1); //da mettere incertezza di GPS, DVL e AHRS
	process_noise << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9; //da definire	

	MatrixXf P = P0.asDiagonal(); //Covariance
	MatrixXf Q = process_noise.asDiagonal(); //Process Noise Cov.

	while(ros::ok())
	{
		ros::spinOnce();	//ricevo i dati dai sensori

		//Rotation matrix
		Matrix3f J1= Jacobian_RPY(eta2);

		/*PREDICTION --> process noise = 0 */
		eta1 = eta1 + T*J1*ni1;
		ni1= ni1;
		eta2= eta2; 

		VectorXf state_pred(n);
		state_pred << eta1, ni1, eta2;

		/*ROS_WARN("Stato Predetto:");
		cout << state_pred << "\n" << endl;*/

		MatrixXf F = vileMatriceF(eta2, ni1);
		MatrixXf D = vileMatriceD();

		P = F*P*F.transpose() + D*Q*D.transpose();
		//cout << P << endl;

		/*CORRECTION --> sensor noise = 0*/
		//Assegnazione dinamica:
		//Ordine: (USBL/GPS) - DVL - DEPTH - AHRS
		VectorXf measures; //max 10
		VectorXf sensor_function; //max 10
		MatrixXf H(0, n); //max 10x9
		MatrixXf M(0, 13); //max 10x13

		
		if (gps_obj.get_underWater())
		{
			sensor_noise << pow(0.01, 2), pow(deg2Rad(0.1), 2), pow(deg2Rad(0.1), 2), 0.4, 0.5, 0.6, 4*pow(10, -6), pow(deg2Rad(0.03),2),  pow(deg2Rad(0.03),2), deg2Rad(1), 0.1, 0.1, 0.1;

			if(usbl_obj.get_isNew())
			{
				int length = measures.size();
				VectorXf meas = measures; //vettori di comodo
				VectorXf s = sensor_function;
				MatrixXf h = H;
				MatrixXf m = M;

				measures.conservativeResize(length + 3);
				sensor_function.conservativeResize(length + 3);
				H.conservativeResize(length +3, n);
				M.conservativeResize(length +3, 13);

				measures << meas, ros2eigen(usbl_obj.get_data());
				sensor_function << s, usbl_obj.function(eta1, eta2);

				H << h, usbl_obj.get_H(eta1, eta2);
				M << m, usbl_obj.get_M();
				usbl_obj.set_isOld();
			}

		}

		else 
		{
			sensor_noise << 1, 1, 1, 0.4, 0.5, 0.6, 4*pow(10, -6), pow(deg2Rad(0.03),2),  pow(deg2Rad(0.03),2), deg2Rad(1), 0.1, 0.1, 0.1;

			if(gps_obj.get_isNew())
			{
				int length = measures.size();
				VectorXf meas = measures; //vettori di comodo
				VectorXf s = sensor_function;
				MatrixXf h = H;
				MatrixXf m = M;

				measures.conservativeResize(length + 3);
				sensor_function.conservativeResize(length + 3);
				H.conservativeResize(length +3, n);
				M.conservativeResize(length +3, 13);

				measures << meas, lla2ned(ros2eigen(gps_obj.get_data()), ned_lla_0);
				sensor_function << s, gps_obj.function(eta1);

				H << h, gps_obj.get_H();
				M << m, gps_obj.get_M();
				gps_obj.set_isOld();
			}

		}

		if(dvl_obj.get_isNew() && gyro_obj.get_isNew()) //da chiedere
		{
			int length = measures.size();
			VectorXf meas = measures; //vettori di comodo
			VectorXf s = sensor_function;
			MatrixXf h = H;
			MatrixXf m = M;

			measures.conservativeResize(length + 3);
			sensor_function.conservativeResize(length + 3);
			H.conservativeResize(length +3, n);
			M.conservativeResize(length +3, 13);

			//Compensazione:
			VectorXf dvl_comp = ros2eigen(dvl_obj.get_data()) - dvl_obj.get_Rsb() * SS(ros2eigen(gyro_obj.get_data())) *dvl_obj.get_pbody();

			measures << meas, dvl_comp;
			sensor_function << s, dvl_obj.function(ni1);
			H << h, dvl_obj.get_H(); 
			M << m, dvl_obj.get_M();			
			dvl_obj.set_isOld();
			gyro_obj.set_isOld();
		}

		if(depth_obj.get_isNew())
		{
			int length = measures.size();
			MatrixXf h = H;
			MatrixXf m = M;

			measures.conservativeResize(length + 1);
			sensor_function.conservativeResize(length + 1);
			H.conservativeResize(length +1, n);
			M.conservativeResize(length +1, 13);
			
			measures(length) = depth_obj.get_data();
			sensor_function(length) = depth_obj.function(eta1);
			H << h, depth_obj.get_H();
			M << m, depth_obj.get_M();

			depth_obj.set_isOld();
		}

		if(ahrs_obj.get_isNew())
		{
			int length = measures.size();
			VectorXf meas = measures; //vettori di comodo
			VectorXf s = sensor_function;
			MatrixXf h = H;
			MatrixXf m = M;

			measures.conservativeResize(length + 3);
			sensor_function.conservativeResize(length + 3);
			H.conservativeResize(length+3, n);
			M.conservativeResize(length+3, 13);

			measures << meas, ros2eigen(ahrs_obj.get_data());
			sensor_function << s, ahrs_obj.function(eta2);

			H << h, ahrs_obj.get_H();
			M << m, ahrs_obj.get_M();

			ahrs_obj.set_isOld();
		}

		MatrixXf R = sensor_noise.asDiagonal(); //Sensor Noise Cov.

		int p= measures.size();
		/*ROS_INFO("Dimensione delle misure: %d\n", p);
		ROS_WARN("Misure\n");
		cout << measures  << "\n" << endl;
		ROS_WARN("h(x, 0)\n");
		cout << sensor_function  << "\n" << endl;*/
		VectorXf e(p);
		e = measures - sensor_function;
		/*ROS_WARN("Matrici H e M \n");
		cout << H << "\n" << endl;
		cout << M << "\n" << endl;*/

		//Correction Algorithm
		MatrixXf S = H*P*H.transpose() + M*R*M.transpose(); //pxp
		MatrixXf L = P*H.transpose()*S.inverse(); //nxp

		/*cout << S << "\n" << endl;
		cout << S.inverse() << "\n" << endl;
		cout << S.completeOrthogonalDecomposition().pseudoInverse() << endl;*/


		VectorXf state_corr = state_pred + L*e;
		//cout << L << endl;
		//cout << e << endl;
		//cout << L*e << endl;
		P = P -L*S*L.transpose();

		/*ROS_WARN("Stato Corretto\n");
		cout << state_corr  << "\n" << endl;
		cout << P << endl;*/

		eta1 << state_corr(0), state_corr(1), state_corr(2);
		ni1 << state_corr(3), state_corr(4), state_corr(5);
		eta2 << state_corr(6), state_corr(7), state_corr(8);

		Vector3f lld_test = ned2lla(eta1, ned_lla_0);
		Vector3f lld(lld_test(0), lld_test(1), eta1(2));

		print_info(state_corr, lld, eta2, P);

		//Publish
		nav_pkg::Odom odom_msg;
		
		odom_msg.lld=eigen2ros(lld);
		odom_msg.lin_vel=eigen2ros(ni1);
		odom_msg.rpy=eigen2ros(eta2);

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
	cout << clear_small_number(P_corr) << endl;
}

