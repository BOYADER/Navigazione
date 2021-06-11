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
#define n_meas 16

//Init Waypoint
float LAT_0;	//[deg]
float LONG_0;	//[deg]
float ALT_0;	//[m]

int N_init;		//n° misure per init
int N_p;		//n° particelle

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
float proc_p;
float proc_q;
float proc_r;

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
VectorXf prediction_particles(VectorXf state, VectorXf dist_proc);
void print_info(VectorXf state_corr, Vector3f lla, MatrixXf P_corr);
void param_Init(ros::NodeHandle node_obj);

/*SENSOR OBJECTS*/
Sensor gyro_obj; //nota: spiegare perche' Sensor e non Gyro::Sensor
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
	ros::Subscriber sub_ahrs=node_obj.subscribe("/sensor/ahrs", 1, ahrs_callback);
	ros::Subscriber sub_usbl=node_obj.subscribe("/sensor/usbl", 1, usbl_callback);
	ros::Subscriber sub_dvl=node_obj.subscribe("/sensor/dvl", 1, dvl_callback);
	ros::Subscriber sub_depth=node_obj.subscribe("/sensor/depth", 1, depth_callback);
	ros::Subscriber sub_gps=node_obj.subscribe("/sensor/gps", 1, gps_callback);
	ros::Rate loop_rate(10);	//10 Hz Prediction step

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
	float streak = 0.0;

	Vector3f eta1(0, 0, 0);  
	Vector3f ni1(0, 0, 0);
	Vector3f eta2(0, 0, 0);
	Vector3f ni2(0, 0, 0);


	ROS_WARN("Initialization");
	while (isInit == 0) 
	{
		loop_rate.sleep();
		ros::spinOnce();

		bool valid_meas = gps_obj.get_isNew() && !gps_obj.get_underWater() && ahrs_obj.get_isNew() && depth_obj.get_isNew();

		if (valid_meas)
			++streak;


		ROS_INFO("Streak: %f", streak);
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
			ROS_INFO("ni2_0 {BODY}: [%f, %f, %f]\n", ni2(0), ni2(1), ni2(2));
			ROS_INFO("Inizializzazione completata. Boya Dehr!\n");
		}
	}
	VectorXf process_noise(n);
	VectorXf sensor_noise(n_meas);
	VectorXf P0(n);

	P0 << var_gps, var_gps, var_dep, 0.01, 0.01, 0.01, var_rp,  var_rp, var_y, var_gyro, var_gyro, var_gyro; 
	process_noise << powf(proc_x, 2.0), powf(proc_y, 2.0), powf(proc_z, 2.0), powf(proc_u, 2.0), powf(proc_v, 2.0), powf(proc_w, 2.0), powf(proc_roll, 2.0),  powf(proc_pitch, 2.0),  powf(proc_yaw, 2.0), powf(proc_p, 2.0), powf(proc_q, 2.0), powf(proc_r, 2.0);
	sensor_noise << var_range, var_be, var_be, 0, 0, 0, var_dep, var_rp,  var_rp, var_y, var_gyro, var_gyro, var_gyro, var_gps, var_gps, var_gps;

	MatrixXf P = P0.asDiagonal(); //Covariance
	MatrixXf Q = process_noise.asDiagonal(); //Process Noise Cov.

	VectorXf x0(n);
	x0 << eta1, ni1, eta2, ni2;

	EigenMultivariateNormal<float> normX_solver(x0, P);
	MatrixXf particles = normX_solver.samples(N_p);
	VectorXd pesi(N_p);
	pesi.setOnes();
	pesi *= (1/ (double) N_p);

	//cout << particles << endl;

	int i = 0; //per iterazioni

	while(ros::ok())
	{
		ros::spinOnce();	//ricevo i dati dai sensori

		/*PREDIZIONE*/
		VectorXf proc_mean(n);
		proc_mean.setZero();
		EigenMultivariateNormal<float> normW_solver(proc_mean, Q); //genero realizz. di w
		MatrixXf w_real = normW_solver.samples(N_p); //n x N_p

		//Applico f() alle particelle
		for( i=0; i < N_p; i++)
		{
			particles.col(i) = prediction_particles(particles.col(i), w_real.col(i));
		}

		/*MatrixXf eta1_particle(3, N_p);
		MatrixXf eta2_particle(3, N_p);
		eta1_particle = particles.block(0, 0, 3, N_p);
		eta2_particle = particles.block(6, 0, 3, N_p);

		MatrixXf ni1_particle(3, N_p);
		MatrixXf ni2_particle(3, N_p);
		ni1_particle = particles.block(3, 0, 3, N_p);
		ni2_particle = particles.block(9, 0, 3, N_p);*/

		//calcolo della media campionaria
		VectorXf state_pred(n);
		state_pred.setZero();

		//TO DO: MEDIA ANGOLI

		for( i =0; i < N_p; i++)
		{
			state_pred  += pesi(i) * particles.col(i);
		}

		for( i =0; i < N_p; i++)
		{
			VectorXf scarto = particles.col(i) - state_pred;
			P  += pesi(i) * scarto * scarto.transpose();
		}

		/*ROS_WARN("Stato predetto:");
		cout << state_pred << "\n" << endl;
		ROS_WARN("Matrice di Covarianza:");
		cout << P << "\n" << endl;*/

		MatrixXf eta1_particle(3, N_p);
		MatrixXf eta2_particle(3, N_p);
		eta1_particle = particles.block(0, 0, 3, N_p);
		eta2_particle = particles.block(6, 0, 3, N_p);

		MatrixXf ni1_particle(3, N_p);
		MatrixXf ni2_particle(3, N_p);
		ni1_particle = particles.block(3, 0, 3, N_p);
		ni2_particle = particles.block(9, 0, 3, N_p);

		eta1 << state_pred(0), state_pred(1), state_pred(2);
		ni1 << state_pred(3), state_pred(4), state_pred(5);
		eta2 << state_pred(6), state_pred(7), state_pred(8);
		ni2 << state_pred(9), state_pred(10), state_pred(11);

		Vector3f bias_std(0.01, 0.01, 0.01);
		Vector3f dev_dvl = dvl_perc*ni1 + bias_std;

		sensor_noise(3) = powf(dev_dvl(0), 2.0);
		sensor_noise(4) = powf(dev_dvl(1), 2.0);
		sensor_noise(5) = powf(dev_dvl(2), 2.0);		

		/*CORREZIONE*/
		MatrixXf R = sensor_noise.asDiagonal(); //Sensor Noise Cov.
		VectorXf measures;
		MatrixXf sensor_function(0, N_p);
		int n_eff =0;

		if(usbl_obj.get_isNew())
		{
			int length = measures.size();
			VectorXf meas = measures; //vettori di comodo
			MatrixXf s1 = sensor_function;
			MatrixXf s2(3, N_p); 

			measures.conservativeResize(length + 3);
			sensor_function.conservativeResize(length + 3, N_p);

			measures << meas, ros2eigen(usbl_obj.get_data());

			for(i=0; i < N_p; i++)
			{
				s2.col(i) = usbl_obj.function(eta1_particle.col(i), eta2_particle.col(i));
			}

			sensor_function << s1, s2;
			//ROS_INFO("USBL\n");
		}

		if(dvl_obj.get_isNew()) 
		{
			int length = measures.size();
			VectorXf meas = measures; //vettori di comodo
			MatrixXf s1 = sensor_function;
			MatrixXf s2(3, N_p); 

			measures.conservativeResize(length + 3);
			sensor_function.conservativeResize(length + 3, N_p);

			measures << meas, ros2eigen(dvl_obj.get_data());

			for(i=0; i < N_p; i++)
			{
				s2.col(i) = dvl_obj.function_PF(ni1_particle.col(i), ni2_particle.col(i));
			}

			sensor_function << s1, s2;
				
			dvl_obj.set_isOld();
			//ROS_INFO("DVL\n");
		}

		if(depth_obj.get_isNew())
		{
			int length = measures.size();

			measures.conservativeResize(length + 1);
			sensor_function.conservativeResize(length + 1, N_p);

			measures(length) = depth_obj.get_data();

			for(i= 0; i < N_p; i++)
			{
				sensor_function(length, i) = depth_obj.function(eta1_particle.col(i));
			}
			depth_obj.set_isOld();
			//ROS_INFO("DEPTH\n");
		}

		if(ahrs_obj.get_isNew())
		{
			int length = measures.size();
			VectorXf meas = measures; //vettori di comodo
			MatrixXf s1 = sensor_function;
			MatrixXf s2(6, N_p); 

			measures.conservativeResize(length + 6);
			sensor_function.conservativeResize(length + 6, N_p);

			measures << meas, ros2eigen(ahrs_obj.get_data()), ros2eigen(gyro_obj.get_data());

			for(i=0; i < N_p; i++)
			{
				s2.col(i) << ahrs_obj.function(eta2_particle.col(i)), ni2_particle.col(i);
			}

			sensor_function << s1, s2;

			ahrs_obj.set_isOld();
			gyro_obj.set_isOld();
			//ROS_INFO("AHRS\n");
		}

		if(gps_obj.get_isNew() && !gps_obj.get_underWater())
		{
			int length = measures.size();
			VectorXf meas = measures; //vettori di comodo
			MatrixXf s1 = sensor_function;
			MatrixXf s2(3, N_p); 

			measures.conservativeResize(length + 3);
			sensor_function.conservativeResize(length + 3, N_p);

			measures << meas, lla2ned(ros2eigen(gps_obj.get_data()), ned_lla_0);

			for(i=0; i < N_p; i++)
			{
				s2.col(i) = gps_obj.function(eta1_particle.col(i));
			}

			sensor_function << s1, s2;

			gps_obj.set_isOld();
			//ROS_INFO("GPS\n");
		}

		int p= measures.size();

		MatrixXf error_particles(p, N_p); 
		
		for(i = 0; i < N_p; i++)
		{
			error_particles.col(i) = measures - sensor_function.col(i);
		}

		if(usbl_obj.get_isNew())
		{
			//norm. -pi / pi
			for(i= 0; i < N_p; i++)
			{
				for(int j = 1; j <3 ; j++)
				{
					VectorXf e = error_particles.col(i);
					e(j) = atan2(sin(e(j)), cos(e(j)));
				}
			}
			usbl_obj.set_isOld();
		}

		//Calcolo dei pesi: w = likelyhood
		for(i=0; i < N_p; i++)
		{
			pesi(i) = likelyhood(error_particles.col(i), R);
		}

		//cout << pesi << endl;

		break;

		//Publish
		nav_pkg::Odom odom_msg;
		
		//odom_msg.lld=eigen2ros(lld);
		odom_msg.lld=eigen2ros(eta1);
		odom_msg.lin_vel=eigen2ros(ni1);
		odom_msg.rpy=eigen2ros(eta2);

		pub.publish(odom_msg);
		loop_rate.sleep();
	}

return 0;
}

VectorXf prediction_particles(VectorXf state, VectorXf dist_proc)
{
	Vector3f x1(state(0), state(1), state(2)); //eta1
	Vector3f x2(state(3), state(4), state(5)); //ni1
	Vector3f x3(state(6), state(7), state(8)); //eta2
	Vector3f x4(state(9), state(10), state(11)); //ni2

	Vector3f w1(dist_proc(0), dist_proc(1), dist_proc(2)); 
	Vector3f w2(dist_proc(3), dist_proc(4), dist_proc(5)); 
	Vector3f w3(dist_proc(6), dist_proc(7), dist_proc(8)); 
	Vector3f w4(dist_proc(9), dist_proc(10), dist_proc(11)); 

	Matrix3f J1 = Jacobian_RPY(x3);
	Matrix3f J2 = Jacobian2(x3);

	x1 = x1 + T*J1*x2 + T*w1;
	x2 = x2 + T*w2;
	x3 = x3 + T*J2*x4 + T*w3;
	x4 = x4 + T*w4;

	//wrap2pi x3

	for(int i = 0; i < 3; i++)
	{
		x3(i) = atan2(sin(x3(i)), cos(x3(i)));
	}

	VectorXf state_pred(n);
	state_pred << x1, x2, x3, x4;
	return state_pred;
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

	//ni2
	node_obj.getParam("/Tuning_Q/ni2/p", proc_p);
	node_obj.getParam("/Tuning_Q/ni2/q", proc_q);
	node_obj.getParam("/Tuning_Q/ni2/r", proc_r);

	node_obj.getParam("/N_init", N_init);
	node_obj.getParam("N_particles", N_p);

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
