/****PROVA 2****/
#include <ostream>
#include <iostream>
#include </home/daniele/eigen/Eigen/Eigen>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"

using namespace std;
using namespace Eigen;

//clear matrix 
float tolerance = 1*pow(10, -5);
MatrixXf clear_small_number(MatrixXf matrix, float tol);

geometry_msgs::Vector3 eigen2ros(Vector3f v);
Vector3f ros2eigen(geometry_msgs::Vector3 v_msg);

Matrix3f Jacobian_RPY(Vector3f rpy);
Matrix3f SS(Vector3f v);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "prova2_eigen");
	ros::NodeHandle node_obj;

	//while(ros::ok())
	//{
		MatrixXf m= MatrixXf::Random(3, 3);
		Vector3f rpy_angle(1, 2, 3);
        VectorXf v(6);
		v << 1, 2, 3, 4, 5, 6;
		MatrixXf m_inv=m.inverse();
	//	MatrixXf test= m*m_inv;

		cout << m << endl;
		cout << m(0, 0) << endl;
        ROS_INFO("\nTEST EIGEN2\n");
        cout << v << endl;
		cout << m_inv << endl;
		cout << clear_small_number(m*m_inv, tolerance) << endl;

		//Test sull'aggiornamento della P
		MatrixXf P= MatrixXf::Random(6, 6);
		MatrixXf F= MatrixXf::Random(6, 6);
		MatrixXf Q = v.asDiagonal();
		
		//MatrixXf test= F*P*(F.transpose()) + Q;
		//cout << test << endl;

		Matrix3f J1= Jacobian_RPY(rpy_angle);
		cout << J1 << endl;

		Matrix3f test2= SS(rpy_angle);
		cout << test2 << endl;

		geometry_msgs::Vector3 ahrs;
		ahrs.x=1;
		ahrs.y=2;
		ahrs.z=3;

		cout << ros2eigen(ahrs) << endl;

	//}


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