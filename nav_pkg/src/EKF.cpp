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
#define n 9 		//dimension State Space
#define n_meas 16	//n° max di misure disponibili

bool maha_enable = 0;

//Init Waypoint
float LAT_0;	//[deg]
float LONG_0;	//[deg]
float ALT_0;	//[m]

int N_init;		//n° misure per init

//Deviaz. standard Disturbo di processo
float proc_x;
float proc_y;
float proc_z;
float proc_u;
float proc_v;
float proc_w;
float proc_roll;
float proc_pitch;
float proc_yaw;

//Deviaz. Standard dei Sensori
float dev_gps;	//[m]
float dev_r;	//[m]
float dev_be;	//[deg]
float dvl_perc;	//[%] su vel
float dev_rp;	//[deg]
float dev_y;	//[deg]
float dev_z;	//[m]
float dev_gyro;	//[rad/s]

/*------------CUSTOM FUNCTION DECLARATION------------*/
void print_info(VectorXf state_corr, Vector3f lla, MatrixXf P_corr);
void param_Init(ros::NodeHandle node_obj);

/*SENSOR OBJECTS*/
Sensor gyro_obj;
Depth depth_obj;
Usbl usbl_obj;
Dvl dvl_obj(0.37, 0, 0.3);
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
}

void usbl_callback(const nav_pkg::usbl::ConstPtr& msg)
{
	usbl_obj.set_data(msg->pos);
	usbl_obj.set_id(msg->counter);
	usbl_obj.set_isNew();
}

void depth_callback(const nav_pkg::depth::ConstPtr& msg)
{
	depth_obj.set_data(msg->z);
	depth_obj.set_id(msg->counter);
	depth_obj.set_isNew();
}

void dvl_callback(const nav_pkg::dvl::ConstPtr& msg)
{
	dvl_obj.set_data(msg->lin_vel);
	dvl_obj.set_id(msg->counter);
	dvl_obj.set_isNew();
}

void gps_callback(const nav_pkg::gps::ConstPtr& msg)
{
	gps_obj.set_data(msg->lla);
	gps_obj.set_id(msg->counter);
	gps_obj.set_isNew();
	gps_obj.set_underWater(msg->under_water);
}

/*------------MAIN------------*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "EKF");
	ros::NodeHandle node_obj;
	
	ros::Publisher pub =  node_obj.advertise<nav_pkg::Odom>("/odom",10);
	ros::Subscriber sub_ahrs=node_obj.subscribe("/sensor/ahrs", 1, ahrs_callback);
	ros::Subscriber sub_usbl=node_obj.subscribe("/sensor/usbl", 1, usbl_callback);
	ros::Subscriber sub_dvl=node_obj.subscribe("/sensor/dvl", 1, dvl_callback);
	ros::Subscriber sub_depth=node_obj.subscribe("/sensor/depth", 1, depth_callback);
	ros::Subscriber sub_gps=node_obj.subscribe("/sensor/gps", 1, gps_callback);
	ros::Rate loop_rate(10);	//10 Hz Prediction step

	ros::Publisher pub2 = node_obj.advertise<geometry_msgs::Vector3>("/pos_ned", 10); //utile per plot

	//Carico i parametri da mission.yaml
	param_Init(node_obj);

	Vector3f ned_lla_0(LAT_0, LONG_0, ALT_0);

	float var_range = powf(dev_r, 2.0);
	float var_be = powf(deg2Rad(dev_be), 2.0);
	float var_dep = powf(dev_z, 2.0);
	float var_rp = powf(deg2Rad(dev_rp), 2.0);
	float var_y = powf(deg2Rad(dev_y), 2.0);
	float var_gyro = powf(dev_gyro, 2.0);
	float var_gps = powf(dev_gps, 2.0);

	//init useful tool
	static int isInit = 0;
	int streak = 0.0;

	Vector3f eta1(0, 0, 0);  
	Vector3f ni1(0, 0, 0);
	Vector3f eta2(0, 0, 0);

	ROS_WARN("Initialization"); 
	while (isInit == 0) 
	{
		loop_rate.sleep();
		ros::spinOnce();

		bool valid_meas = gps_obj.get_isNew() && !gps_obj.get_underWater() && ahrs_obj.get_isNew() && depth_obj.get_isNew();

		if (valid_meas)
			++streak;


		ROS_INFO("Streak: %d", streak);
		ROS_INFO("GPS isNew: %d", gps_obj.get_isNew());
		ROS_INFO("AHRS isNew: %d", ahrs_obj.get_isNew());
		ROS_INFO("DEPTH isNew: %d", depth_obj.get_isNew());


		if (streak >= N_init)
		{
			eta1 << lla2ned(ros2eigen(gps_obj.get_data()), ned_lla_0);
			eta1(2) = depth_obj.get_data();
			eta2 << ros2eigen(ahrs_obj.get_data());

			isInit = 1;
			ROS_INFO("eta1_0 {NED}: [%f, %f, %f]\n", eta1(0), eta1(1), eta1(2));
			ROS_INFO("ni1_0 {BODY}: [%f, %f, %f]\n", ni1(0), ni1(1), ni1(2));
			ROS_INFO("eta2_0 {NED}: [%f, %f, %f]\n", eta2(0), eta2(1), eta2(2));
			ROS_INFO("Inizializzazione completata. Boya Dehr!\n");
		}
	}
	
	VectorXf process_noise(n);
	VectorXf sensor_noise(n_meas);
	VectorXf P0(n);

	P0 << var_gps, var_gps, var_dep, 0.01, 0.01, 0.01, var_rp,  var_rp, var_y; 
	process_noise << powf(proc_x, 2.0), powf(proc_y, 2.0), powf(proc_z, 2.0), powf(proc_u, 2.0), powf(proc_v, 2.0), powf(proc_w, 2.0), powf(proc_roll, 2.0),  powf(proc_pitch, 2.0),  powf(proc_yaw, 2.0);
	sensor_noise << 0.5*var_range, 0.5*var_be, 0.5*var_be, 0, 0, 0, var_dep, var_rp,  var_rp, var_y, var_gyro, var_gyro, var_gyro, var_gps, var_gps, var_gps;

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

		MatrixXf F = vileMatriceF(eta2, ni1);
		MatrixXf D = vileMatriceD();

		P = F*P*F.transpose() + D*Q*D.transpose();

		VectorXf measures; //max 13
		VectorXf sensor_function; //max 13
		VectorXf e;
		MatrixXf H(0, n); //max 13x9
		MatrixXf M(0, n_meas); //max 13x16

		Vector3f bias_std(0.01, 0.01, 0.01);
		Vector3f dev_dvl = dvl_perc*ni1 + bias_std;

		sensor_noise(3) = powf(dev_dvl(0), 2.0);
		sensor_noise(4) = powf(dev_dvl(1), 2.0);
		sensor_noise(5) = powf(dev_dvl(2), 2.0);

		MatrixXf R = sensor_noise.asDiagonal(); //Sensor Noise Cov.

		/*Correzione dinamica*/
		/* Ogni if(), che si verifica quando c'e' una nuova misura,					*
		 * aggiorna smeasures e sensor_function e la matrice H con i nuovi valori, 	*
		 * la misura predetta e le corrispondenti righe della matrice H				*
		 * associata al sensore.													*/

		if(usbl_obj.get_isNew())
		{
			int length = measures.size();
			VectorXf meas = measures; //vettori di comodo
			VectorXf s = sensor_function;
			MatrixXf h = H;
			MatrixXf m = M;

			VectorXf meas_sing = ros2eigen(usbl_obj.get_data()); //cerco anche nelle misure la singolarita'
			int n_sing_meas = 3;
			int n_sing_pred = usbl_obj.function(eta1, eta2).size(); //2: Singolarita', 3: Funzionamento regolare

			if((abs(meas_sing(2)) > M_PI_2 - usbl_tol) &&  (abs(meas_sing(2)) < M_PI_2 + usbl_tol)) //~ M_PI/2
			{
				//ROS_WARN("USBL: Singolarita' (Misura)! \n");
				n_sing_meas = 2;
			}

			int n_eff = min(n_sing_meas, n_sing_pred);

			measures.conservativeResize(length + n_eff);
			sensor_function.conservativeResize(length + n_eff);
			e.conservativeResize(length +n_eff);
			H.conservativeResize(length +n_eff, n);
			M.conservativeResize(length +n_eff, n_meas);

			if(n_sing_pred == 2 || n_sing_meas == 2) 
			{
				//altri vettori di comodo per non appesantire il calcolo
				MatrixXf H_sing = usbl_obj.get_H(eta1, eta2);
				MatrixXf M_sing = usbl_obj.get_M();

				measures << meas, meas_sing(0), meas_sing(2); //estraggo solo range ed elevation

				if(n_sing_pred == 3)
					sensor_function << s, usbl_obj.function(eta1, eta2).row(0), usbl_obj.function(eta1, eta2).row(2);

				else if(n_sing_pred == 2)
					sensor_function << s, usbl_obj.function(eta1, eta2);			

				H << h, H_sing.row(0), H_sing.row(2);
				M << m, M_sing.row(0), M_sing.row(2);
			}

			else
			{
				measures << meas, ros2eigen(usbl_obj.get_data());
				sensor_function << s, usbl_obj.function(eta1, eta2);

				H << h, usbl_obj.get_H(eta1, eta2);
				M << m, usbl_obj.get_M();
			}

			e = measures - sensor_function;

			for(int i = 1; i < n_eff; i++)
			{
				e(length + i) = atan2(sin(e(length + i)), cos(e(length + i)));
			}

			if(mahalanobis(e, H*P*H.transpose() + M*R*M.transpose()) && (n_eff ==2) && maha_enable)
			{
				ROS_WARN("MAHALANOBIS");
				measures.resize(length);
				sensor_function.resize(length);
				e.resize(length);
				H.resize(length, n);
				M.resize(length, n_meas);
			}
			usbl_obj.set_isOld();
		}

		if(dvl_obj.get_isNew()) 
		{
			int length = measures.size();
			VectorXf meas = measures; //vettori di comodo
			VectorXf s = sensor_function;
			VectorXf err = e;
			MatrixXf h = H;
			MatrixXf m = M;

			measures.conservativeResize(length + 3);
			sensor_function.conservativeResize(length + 3);
			e.conservativeResize(length + 3);
			H.conservativeResize(length +3, n);
			M.conservativeResize(length +3, n_meas);

			//Compensazione:
			VectorXf dvl_comp = ros2eigen(dvl_obj.get_data()) - dvl_obj.get_Rsb() * SS(ros2eigen(gyro_obj.get_data())) *dvl_obj.get_pbody();

			measures << meas, dvl_comp;
			sensor_function << s, dvl_obj.function(ni1);
			e << err, dvl_comp - dvl_obj.function(ni1); 
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
			e.conservativeResize(length + 1);
			H.conservativeResize(length +1, n);
			M.conservativeResize(length +1, n_meas);
			measures(length) = depth_obj.get_data();
			sensor_function(length) = depth_obj.function(eta1);
			e(length) = depth_obj.get_data() - depth_obj.function(eta1);
			H << h, depth_obj.get_H();
			M << m, depth_obj.get_M();

			depth_obj.set_isOld();
		}

		if(ahrs_obj.get_isNew())
		{
			int length = measures.size();
			VectorXf meas = measures; //vettori di comodo
			VectorXf s = sensor_function;
			VectorXf err = e;
			MatrixXf h = H;
			MatrixXf m = M;

			measures.conservativeResize(length + 3);
			sensor_function.conservativeResize(length + 3);
			e.conservativeResize(length+3);
			H.conservativeResize(length+3, n);
			M.conservativeResize(length+3, n_meas);

			measures << meas, ros2eigen(ahrs_obj.get_data());
			sensor_function << s, ahrs_obj.function(eta2);

			Vector3f e_ahrs=ros2eigen(ahrs_obj.get_data()) - ahrs_obj.function(eta2);

			for(int i = 0; i < 3; i++)
			{
				e_ahrs(i) = atan2(sin(e_ahrs(i)), cos(e_ahrs(i)));
			}

			e << err, e_ahrs;

			H << h, ahrs_obj.get_H();
			M << m, ahrs_obj.get_M();

			ahrs_obj.set_isOld();
			gyro_obj.set_isOld();
		}

		if(gps_obj.get_isNew() && !gps_obj.get_underWater())
		{
			int length = measures.size();
			VectorXf meas = measures; //vettori di comodo
			VectorXf s = sensor_function;
			VectorXf err = e;
			MatrixXf h = H;
			MatrixXf m = M;

			measures.conservativeResize(length + 3);
			sensor_function.conservativeResize(length + 3);
			e.conservativeResize(length + 3);
			H.conservativeResize(length +3, n);
			M.conservativeResize(length +3, n_meas);

			measures << meas, lla2ned(ros2eigen(gps_obj.get_data()), ned_lla_0);
			sensor_function << s, gps_obj.function(eta1);
			e << err, lla2ned(ros2eigen(gps_obj.get_data()), ned_lla_0) - gps_obj.function(eta1);

			H << h, gps_obj.get_H();
			M << m, gps_obj.get_M();
			gps_obj.set_isOld();
		}
		
		int p= measures.size();

		//Correction Algorithm
		MatrixXf S = H*P*H.transpose() + M*R*M.transpose(); //pxp
		MatrixXf L = P*H.transpose()*S.inverse(); //nxp

		VectorXf state_corr = state_pred + L*e;
		P = P -L*S*L.transpose();

		eta1 << state_corr(0), state_corr(1), state_corr(2);
		ni1 << state_corr(3), state_corr(4), state_corr(5);
		eta2 << state_corr(6), state_corr(7), state_corr(8);

		for(int i = 0; i < 3; i++)
		{
			eta2(i) = atan2(sin(eta2(i)), cos(eta2(i)));
		}

		Vector3f lld_test = ned2lla(eta1, ned_lla_0);
		Vector3f lld(lld_test(0), lld_test(1), eta1(2));

		print_info(state_corr, lld, P);

		//Publish
		nav_pkg::Odom odom_msg;
		
		odom_msg.lld=eigen2ros(lld);
		odom_msg.lin_vel=eigen2ros(ni1);
		odom_msg.rpy=eigen2ros(eta2);

		//pubblico anche in {NED} per rqt_plot
		geometry_msgs::Vector3 pos_ned = eigen2ros(eta1); 

		pub.publish(odom_msg);
		pub2.publish(pos_ned);
		loop_rate.sleep();
	}

return 0;
}

void print_info(VectorXf state_corr, Vector3f lla, MatrixXf P_corr)
{
	ROS_INFO("|********EKF RESULT*********|\n");

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

	/*Tuning di Q*/
	//eta1
	node_obj.getParam("/Tuning_Q/eta1/x", proc_x);
	node_obj.getParam("/Tuning_Q/eta1/y", proc_y);
	node_obj.getParam("/Tuning_Q/eta1/z", proc_z);

	//ni1
	node_obj.getParam("/Tuning_Q/ni1/u", proc_u);
	node_obj.getParam("/Tuning_Q/ni1/v", proc_v);
	node_obj.getParam("/Tuning_Q/ni1/w", proc_w);

	//eta2
	node_obj.getParam("/Tuning_Q/eta2/r", proc_roll);
	node_obj.getParam("/Tuning_Q/eta2/p", proc_pitch);
	node_obj.getParam("/Tuning_Q/eta2/y", proc_yaw);

	node_obj.getParam("/N_init", N_init);

	//Deviaz. standard Sensori
	node_obj.getParam("/sensor_dev/gps", dev_gps);
	node_obj.getParam("/sensor_dev/usbl/range", dev_r);
	node_obj.getParam("/sensor_dev/usbl/b_e", dev_be);
	node_obj.getParam("/sensor_dev/dvl", dvl_perc);
	node_obj.getParam("/sensor_dev/depth", dev_z);
	node_obj.getParam("/sensor_dev/ahrs/r_p", dev_rp);
	node_obj.getParam("/sensor_dev/ahrs/yaw", dev_y);
	node_obj.getParam("/sensor_dev/gyro", dev_gyro);
}
