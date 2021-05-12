#include <ostream>
#include <iostream>
#include </home/daniele/eigen/Eigen/Eigen>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "nav_pkg/Odom.h"
#include "nav_pkg/usbl.h"
#include "nav_pkg/ahrs.h"
#include "nav_pkg/imu.h"
#include "nav_pkg/dvl.h"
#include "nav_pkg/depth.h"

using namespace std;
using namespace Eigen;

#define T 0.1

/*CUSTOM FUNCTION DECLARATION*/
//clear matrix 
float tolerance = 1*pow(10, -5);
MatrixXf clear_small_number(MatrixXf matrix, float tol);

//eigen-ros interface
geometry_msgs::Vector3 eigen2ros(Vector3f v);
Vector3f ros2eigen(geometry_msgs::Vector3 v_msg);

//Jacobian and Skew Symmetrix matrix function
Matrix3f Jacobian_RPY(Vector3f rpy);
Matrix3f SS(Vector3f v);

//Navigazione-Controllo interface
geometry_msgs::Vector3 eta1_dot2ni(Vector3f eta1_dot);

//Linearization Matrix
//MatrixXf vileMatriceF(Vector3f rpy, Vector3f gyro);

/*GLOBAL VARIABLES*/
//input: acc - gyro - orientam.
geometry_msgs::Vector3 acceler;
geometry_msgs::Vector3 gyrosc;
geometry_msgs::Vector3 ahrs;

//sens. non inerziali
std_msgs::Float64 depth;
geometry_msgs::Vector3 usbl;
geometry_msgs::Vector3 dvl;

/*CALLBACK FUNCTIONS*/
void imu_callback(const nav_pkg::imu::ConstPtr& msg)
{
	acceler = msg->acc;
	gyrosc = msg->gyro;
	ROS_WARN("IMU DATA: \n");
	ROS_INFO("Accelerometro: [%f, %f, %f]\n", msg->acc.x, msg->acc.y, msg->acc.z);
	ROS_INFO("Giroscopio: [%f, %f, %f]\n", msg->gyro.x, msg->gyro.y, msg->gyro.z);
}

void ahrs_callback(const nav_pkg::ahrs::ConstPtr& msg)
{
	ahrs = msg->rpy;
	ROS_WARN("AHRS DATA: \n");
	ROS_INFO("Orientamento: [%f, %f, %f]\n", msg->rpy.x, msg->rpy.y, msg->rpy.z);
}

void usbl_callback(const nav_pkg::usbl::ConstPtr& msg)
{
	usbl = msg->pos;
	ROS_WARN("USBL DATA: \n");
	ROS_INFO("Posizione: [%f, %f]\n", msg->pos.x, msg->pos.y);
}

void depth_callback(const nav_pkg::depth::ConstPtr& msg)
{
	depth.data = msg->z;
	ROS_WARN("DEPTH DATA: \n");
	ROS_INFO("Profondita: %f\n", msg->z);
}

void dvl_callback(const nav_pkg::dvl::ConstPtr& msg)
{
	dvl = msg->lin_vel;
	ROS_WARN("DVL DATA: \n");
	ROS_INFO("Velocita: [%f, %f, %f]\n", msg->lin_vel.x, msg->lin_vel.y, msg->lin_vel.z);
}


/*MAIN*/
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
	ros::Rate loop_rate(1/T);	//10 Hz Prediction step

	//state vector: Init at x0
	Vector3f eta1(1, 2, 3);
	Vector3f eta1_dot(4, 5, 6); 
	VectorXf v(6);
	v << 1, 2, 3, 4, 5, 6;
	MatrixXf P = v.asDiagonal(); 

	while(ros::ok())
	{
		ros::spinOnce();	//ricevo i dati dai sensori

		//Useful matrix
		Matrix3f J1= Jacobian_RPY(ros2eigen(ahrs));
		Matrix3f skew_sym= SS(ros2eigen(gyrosc));

		//prediction
		eta1 = eta1 + T*eta1_dot;
		eta1_dot= eta1_dot + T*J1*skew_sym*J1.transpose()*eta1_dot + J1*ros2eigen(acceler);

		cout << eta1 << endl;
		cout << eta1_dot << endl;
		//TO DO: Aggiornamento della Covarianza 

		nav_pkg::Odom odom_msg;
		
		odom_msg.lla=eigen2ros(eta1);
		odom_msg.lin_vel=eigen2ros(eta1_dot);		//Se passo eta1_dot {NED}
		//odom_msg.lin_vel=eta1_dot2ni(eta1_dot, J1);		//Se passo ni1 {BODY}
		odom_msg.rpy=ahrs;

		/*odom_msg.lla.x=1;
		odom_msg.lla.y=2;
		odom_msg.lla.z=3;
		odom_msg.rpy.x=1;
		odom_msg.rpy.y=2;
		odom_msg.rpy.z=3;*/

		/*ROS_WARN("Stima: \n");
		ROS_INFO("Latitudine: %f\n",odom_msg.lla.x);
		ROS_INFO("Longitudine: %f\n",odom_msg.lla.y);
		ROS_INFO("Quota: %f\n",odom_msg.lla.z);
		ROS_INFO("Roll:%f\n",odom_msg.rpy.x);
		ROS_INFO("Pitch:%f\n",odom_msg.rpy.y);
		ROS_INFO("Yaw:%f\n",odom_msg.rpy.z);*/

		pub.publish(odom_msg);
		loop_rate.sleep();
	}

return 0;

}

MatrixXf clear_small_number(MatrixXf matrix, float tol)
{
	for(int i=0; i < matrix.rows(); i++)
	{
		for(int j=0; j < matrix.cols(); j++)
		{
			if(matrix(i, j) < tol)
			matrix(i, j)=0;
		}

	}
	return matrix;
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

Matrix3f Jacobian_RPY(Vector3f rpy) 	//rad
{
	float phi= rpy(0);
	float theta= rpy(1);
	float psi = rpy(2);

    Matrix3f m1;
    m1(0,0) = cos(phi)*cos(theta);
    m1(0,1) = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); 
    m1(0,2) = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    m1(1,0) = sin(phi)*cos(theta);
    m1(1,1) = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
    m1(1,2) = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
    m1(2,0) = -sin(theta);
    m1(2,1) = cos(theta)*sin(psi);
    m1(2,2) = cos(theta)*cos(psi);

    return m1;

}

Matrix3f SS(Vector3f v)
{

    float a = v(0);
    float b = v(1);
    float c = v(2);

    Matrix3f m1(3,3);
    m1(0,0) = 0;
    m1(0,1) = -c; 
    m1(0,2) = b;
    m1(1,0) = c;
    m1(1,1) = 0;
    m1(1,2) = -a;
    m1(2,0) = -b;
    m1(2,1) = a;
    m1(2,2) = 0;

    return m1;

}

geometry_msgs::Vector3 eta1_dot2ni(Vector3f eta1_dot, Matrix3f Rot_matrix)
{
	Vector3f ni1 = (Rot_matrix.transpose())*eta1_dot;
	return eigen2ros(ni1);
}

/*VILI MATRICI*/

//Si riferiscono al modello vecchio pero'.

/*MatrixXf vileMatriceF(Vector3f rpy, Vector3f gyro)
{
	float phi = rpy(1);
	float theta = rpy(2);
	float psi = rpy(3);

	float gyro_x=gyro(1);
	float gyro_y=gyro(2);
	float gyro_z=gyro(3);

	Matrix3f block1;
	block1 = MatrixXf::Identity(3, 3);
	Matrix3f block2;
	block2 = T*block1;
	Matrix3f block3;
	block3 = MatrixXf::Zero(3, 3);
	Matrix3f block4;

	block4(0,0) = 1;
	block4(0,1) = -T*(gyro_z*cos(phi)*cos(theta) - gyro_x*sin(theta) + gyro_y*cos(theta)*sin(phi));
	block4(0, 2)= T*(gyro_y*cos(phi)*cos(psi) - gyro_z*cos(psi)*sin(phi) + gyro_x*cos(theta)*sin(psi) + gyro_z*cos(phi)*sin(psi)*sin(theta) + gyro_y*sin(phi)*sin(psi)*sin(theta));

	block4(1, 0) = T*(gyro_z*cos(phi)*cos(theta) - gyro_x*sin(theta) + gyro_y*cos(theta)*sin(phi));
	block4(1, 1) = 1;
	block4(1, 2) = -T*(gyro_x*cos(psi)*cos(theta) - gyro_y*cos(phi)*sin(psi) + gyro_z*sin(phi)*sin(psi) + gyro_z*cos(phi)*cos(psi)*sin(theta) + gyro_y*cos(psi)*sin(phi)*sin(theta));

	block4(2, 0) = - T*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(gyro_z*sin(theta) + gyro_x*cos(phi)*cos(theta)) - T*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(gyro_y*sin(theta) + gyro_x*cos(theta)*sin(phi)) - T*cos(psi)*cos(theta)^2*(gyro_y*cos(phi) - gyro_z*sin(phi));
	block4(2, 1) = T*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(gyro_z*sin(theta) + gyro_x*cos(phi)*cos(theta)) + T*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(gyro_y*sin(theta) + gyro_x*cos(theta)*sin(phi)) - T*cos(theta)^2*sin(psi)*(gyro_y*cos(phi) - gyro_z*sin(phi));
	block4(2, 2) = 1;

	MatrixXf F(6, 6);
	F << block1, block2, block3, block4;

	cout << F << endl;

	return F;
 
}*/
/*F =
 
[1, 0, 0,                                                                                                                                                                                                                                                                             T,                                                                                                                                                                                                                                                                           0,                                                                                                                                                                 0]
[0, 1, 0,                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                           T,                                                                                                                                                                 0]
[0, 0, 1,                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                           0,                                                                                                                                                                 T]
[0, 0, 0,                                                                                                                                                                                                                                                                             1,                                                                                                                                                                                            -T*(gyro_z*cos(phi)*cos(theta) - gyro_x*sin(theta) + gyro_y*cos(theta)*sin(phi)),  T*(gyro_y*cos(phi)*cos(psi) - gyro_z*cos(psi)*sin(phi) + gyro_x*cos(theta)*sin(psi) + gyro_z*cos(phi)*sin(psi)*sin(theta) + gyro_y*sin(phi)*sin(psi)*sin(theta))]
[0, 0, 0,                                                                                                                                                                                               T*(gyro_z*cos(phi)*cos(theta) - gyro_x*sin(theta) + gyro_y*cos(theta)*sin(phi)),                                                                                                                                                                                                                                                                           1, -T*(gyro_x*cos(psi)*cos(theta) - gyro_y*cos(phi)*sin(psi) + gyro_z*sin(phi)*sin(psi) + gyro_z*cos(phi)*cos(psi)*sin(theta) + gyro_y*cos(psi)*sin(phi)*sin(theta))]
[0, 0, 0, - T*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(gyro_z*sin(theta) + gyro_x*cos(phi)*cos(theta)) - T*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(gyro_y*sin(theta) + gyro_x*cos(theta)*sin(phi)) - T*cos(psi)*cos(theta)^2*(gyro_y*cos(phi) - gyro_z*sin(phi)), T*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(gyro_z*sin(theta) + gyro_x*cos(phi)*cos(theta)) + T*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(gyro_y*sin(theta) + gyro_x*cos(theta)*sin(phi)) - T*cos(theta)^2*sin(psi)*(gyro_y*cos(phi) - gyro_z*sin(phi)),                                                                                                                                                                 1]*/

/*D =
 
[                    0,                                                    0,                                                    0,                                                 0,                                                                                                                         0,                                                                                                                               0,                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           0]
[                    0,                                                    0,                                                    0,                                                 0,                                                                                                                         0,                                                                                                                               0,                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           0]
[                    0,                                                    0,                                                    0,                                                 0,                                                                                                                         0,                                                                                                                               0,                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           0]
[T*cos(psi)*cos(theta), T*cos(psi)*sin(phi)*sin(theta) - T*cos(phi)*sin(psi), T*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)),  T*(y_dot*sin(theta) + z_dot*cos(theta)*sin(psi)),                              T*(z_dot*cos(phi)*cos(psi) - y_dot*cos(theta)*sin(phi) + z_dot*sin(phi)*sin(psi)*sin(theta)),                                   -T*(y_dot*cos(phi)*cos(theta) + z_dot*cos(psi)*sin(phi) - z_dot*cos(phi)*sin(psi)*sin(theta)),               T*(acc_z*cos(phi)*sin(psi) + acc_y*sin(phi)*sin(psi) + acc_y*cos(phi)*cos(psi)*sin(theta) - acc_z*cos(psi)*sin(phi)*sin(theta) - gyro_z*z_dot*cos(phi)*cos(psi) - gyro_y*y_dot*cos(phi)*cos(theta) - gyro_y*z_dot*cos(psi)*sin(phi) + gyro_z*y_dot*cos(theta)*sin(phi) + gyro_y*z_dot*cos(phi)*sin(psi)*sin(theta) - gyro_z*z_dot*sin(phi)*sin(psi)*sin(theta)),  T*(gyro_x*y_dot*cos(theta) - acc_x*cos(psi)*sin(theta) + gyro_y*y_dot*sin(phi)*sin(theta) - gyro_x*z_dot*sin(psi)*sin(theta) + acc_z*cos(phi)*cos(psi)*cos(theta) + acc_y*cos(psi)*cos(theta)*sin(phi) + gyro_z*y_dot*cos(phi)*sin(theta) + gyro_z*z_dot*cos(phi)*cos(theta)*sin(psi) + gyro_y*z_dot*cos(theta)*sin(phi)*sin(psi)),                                                                                                                                                                                                                   -T*(acc_y*cos(phi)*cos(psi) - acc_z*cos(psi)*sin(phi) + acc_x*cos(theta)*sin(psi) - gyro_z*z_dot*sin(phi)*sin(psi) + acc_z*cos(phi)*sin(psi)*sin(theta) + acc_y*sin(phi)*sin(psi)*sin(theta) - gyro_x*z_dot*cos(psi)*cos(theta) + gyro_y*z_dot*cos(phi)*sin(psi) - gyro_z*z_dot*cos(phi)*cos(psi)*sin(theta) - gyro_y*z_dot*cos(psi)*sin(phi)*sin(theta))]
[T*cos(theta)*sin(psi), T*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), T*cos(phi)*sin(psi)*sin(theta) - T*cos(psi)*sin(phi), -T*(x_dot*sin(theta) + z_dot*cos(psi)*cos(theta)),                              T*(z_dot*cos(phi)*sin(psi) + x_dot*cos(theta)*sin(phi) - z_dot*cos(psi)*sin(phi)*sin(theta)),                                   -T*(z_dot*sin(phi)*sin(psi) - x_dot*cos(phi)*cos(theta) + z_dot*cos(phi)*cos(psi)*sin(theta)),              -T*(acc_z*cos(phi)*cos(psi) + acc_y*cos(psi)*sin(phi) + gyro_y*z_dot*sin(phi)*sin(psi) - acc_y*cos(phi)*sin(psi)*sin(theta) + acc_z*sin(phi)*sin(psi)*sin(theta) - gyro_y*x_dot*cos(phi)*cos(theta) + gyro_z*z_dot*cos(phi)*sin(psi) + gyro_z*x_dot*cos(theta)*sin(phi) + gyro_y*z_dot*cos(phi)*cos(psi)*sin(theta) - gyro_z*z_dot*cos(psi)*sin(phi)*sin(theta)), -T*(gyro_x*x_dot*cos(theta) + acc_x*sin(psi)*sin(theta) + gyro_y*x_dot*sin(phi)*sin(theta) - acc_z*cos(phi)*cos(theta)*sin(psi) - acc_y*cos(theta)*sin(phi)*sin(psi) + gyro_z*x_dot*cos(phi)*sin(theta) - gyro_x*z_dot*cos(psi)*sin(theta) + gyro_z*z_dot*cos(phi)*cos(psi)*cos(theta) + gyro_y*z_dot*cos(psi)*cos(theta)*sin(phi)),                                                                                                                                                                                                                    T*(acc_x*cos(psi)*cos(theta) - acc_y*cos(phi)*sin(psi) + acc_z*sin(phi)*sin(psi) + acc_z*cos(phi)*cos(psi)*sin(theta) + acc_y*cos(psi)*sin(phi)*sin(theta) + gyro_y*z_dot*cos(phi)*cos(psi) - gyro_z*z_dot*cos(psi)*sin(phi) + gyro_x*z_dot*cos(theta)*sin(psi) + gyro_z*z_dot*cos(phi)*sin(psi)*sin(theta) + gyro_y*z_dot*sin(phi)*sin(psi)*sin(theta))]
[        -T*sin(theta),                                T*cos(theta)*sin(phi),                                T*cos(phi)*cos(theta),    T*cos(theta)*(y_dot*cos(psi) - x_dot*sin(psi)), - T*x_dot*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - T*y_dot*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)), T*(x_dot*cos(psi)*sin(phi) + y_dot*sin(phi)*sin(psi) + y_dot*cos(phi)*cos(psi)*sin(theta) - x_dot*cos(phi)*sin(psi)*sin(theta)), T*(acc_y*cos(phi)*cos(theta) - acc_z*cos(theta)*sin(phi) + gyro_y*y_dot*sin(phi)*sin(psi) + gyro_z*x_dot*cos(phi)*cos(psi) + gyro_y*x_dot*cos(psi)*sin(phi) + gyro_z*y_dot*cos(phi)*sin(psi) + gyro_y*y_dot*cos(phi)*cos(psi)*sin(theta) - gyro_y*x_dot*cos(phi)*sin(psi)*sin(theta) - gyro_z*y_dot*cos(psi)*sin(phi)*sin(theta) + gyro_z*x_dot*sin(phi)*sin(psi)*sin(theta)), -T*(acc_x*cos(theta) + acc_z*cos(phi)*sin(theta) + acc_y*sin(phi)*sin(theta) - gyro_x*x_dot*sin(psi)*sin(theta) + gyro_x*y_dot*cos(psi)*sin(theta) - gyro_z*y_dot*cos(phi)*cos(psi)*cos(theta) + gyro_z*x_dot*cos(phi)*cos(theta)*sin(psi) - gyro_y*y_dot*cos(psi)*cos(theta)*sin(phi) + gyro_y*x_dot*cos(theta)*sin(phi)*sin(psi)), - x_dot*(T*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(gyro_z*sin(theta) + gyro_x*cos(phi)*cos(theta)) + T*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(gyro_y*sin(theta) + gyro_x*cos(theta)*sin(phi)) - T*cos(theta)^2*sin(psi)*(gyro_y*cos(phi) - gyro_z*sin(phi))) - y_dot*(T*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(gyro_z*sin(theta) + gyro_x*cos(phi)*cos(theta)) + T*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(gyro_y*sin(theta) + gyro_x*cos(theta)*sin(phi)) + T*cos(psi)*cos(theta)^2*(gyro_y*cos(phi) - gyro_z*sin(phi)))]*/