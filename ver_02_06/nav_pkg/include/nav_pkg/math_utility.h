/*----------------MATH UTILITY----------------*/
#include <iostream>
#include <eigen/Eigen/Eigen>

using namespace std;
using namespace Eigen;

/*------------PARAMETERS DEFINITIONS------------*/
#define T 0.1
#define mahalanobis_threshold 20
float tolerance = 1*pow(10, -5);

/*------------CUSTOM FUNCTION DECLARATION------------*/
MatrixXf clear_small_number(MatrixXf matrix);
geometry_msgs::Vector3 eigen2ros(Vector3f v);
Vector3f ros2eigen(geometry_msgs::Vector3 v_msg);

//Jacobian and Skew Symmetrix matrix function
Matrix3f Jacobian_RPY(Vector3f rpy);
Matrix3f Jacobian_RPY(float phi, float theta, float psi);
Matrix3f Jacobian2(Vector3f rpy);
Matrix3f SS(Vector3f v);
double likelyhood(VectorXf mean, MatrixXf cov);
bool mahalanobis(VectorXf error, MatrixXf S);
bool check_DefPos(MatrixXf Cov); //meh, non funziona bene

//Linearization Matrix
MatrixXf vileMatriceF(Vector3f rpy, Vector3f ni1);
MatrixXf vileMatriceD();

/*----------FUNCTION--------*/
MatrixXf clear_small_number(MatrixXf matrix)
{
	for(int i=0; i < matrix.rows(); i++)
	{
		for(int j=0; j < matrix.cols(); j++)
		{
			if(matrix(i, j) < tolerance)
			matrix(i, j)=0;
		}

	}
	return matrix;
}

Matrix3f Jacobian_RPY(float phi, float theta, float psi) 	//rad
{
    Matrix3f m1;
    m1(0,0) = cos(psi)*cos(theta);
    m1(0,1) = cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi); 
    m1(0,2) = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    m1(1,0) = sin(psi)*cos(theta);
    m1(1,1) = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
    m1(1,2) = sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
    m1(2,0) = -sin(theta);
    m1(2,1) = cos(theta)*sin(phi);
    m1(2,2) = cos(theta)*cos(phi);

    return m1;
}

Matrix3f Jacobian_RPY(Vector3f rpy) 	//rad
{
	float phi= rpy(0);
	float theta= rpy(1);
	float psi = rpy(2);

    Matrix3f m1;
    m1(0,0) = cos(psi)*cos(theta);
    m1(0,1) = cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi); 
    m1(0,2) = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    m1(1,0) = sin(psi)*cos(theta);
    m1(1,1) = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
    m1(1,2) = sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
    m1(2,0) = -sin(theta);
    m1(2,1) = cos(theta)*sin(phi);
    m1(2,2) = cos(theta)*cos(phi);

    return m1;
}

Matrix3f Jacobian2(Vector3f rpy)
{
	float phi= rpy(0);
	float theta= rpy(1);
	float psi = rpy(2);

	Matrix3f m;
	m(0, 0) = 1;
	m(0, 1) = sin(phi)*tan(theta);
	m(0, 2) = cos(phi)*tan(theta);
	m(1, 0) = 0;
	m(1, 1) = cos(phi);
	m(1, 2) = -sin(phi);
	m(2, 0) = 0;
	m(2, 1) = sin(phi)/cos(theta);
	m(2, 2) = cos(phi)/cos(theta);

	return m;

}

Matrix3f SS(Vector3f v)
{

    float a = v(0);
    float b = v(1);
    float c = v(2);

    Matrix3f m1;
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

bool check_DefPos(MatrixXf Cov)
{
	LLT<MatrixXf> lltOfA(Cov);

	if (lltOfA.info() == NumericalIssue)
		return 0;

	else 
		return 1;
}

double likelyhood(VectorXf mean, MatrixXf cov)
{
	//Verosimiglianza Gaussiana Multivariabile
	int d = mean.size();
	float coeff, esponente;
	coeff = 1/(sqrt(2.0*powf(M_PI, d)*cov.determinant()));
	esponente = ( mean.transpose()*cov.inverse()*mean);
	esponente *= -0.5;
	double test = coeff*exp(esponente);
	return test;
}

bool mahalanobis(VectorXf error, MatrixXf S)
{
	float dm = sqrt(error.transpose()*S.inverse()*error);
	return dm > mahalanobis_threshold;
}
/*VILI MATRICI*/
MatrixXf vileMatriceF(Vector3f rpy, Vector3f ni1)
{

	float phi = rpy(0);
	float theta = rpy(1);
	float psi = rpy(2);

	float u = ni1(0);
	float v = ni1(1);
	float w = ni1(2);

	MatrixXf block1(3, 3);
	block1 = MatrixXf::Identity(3, 3);
	MatrixXf block2(3, 3);
	block2 = T*Jacobian_RPY(rpy);

	MatrixXf block3(3, 3);
	block3(0, 0) = T*v*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + T*w*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta));
	block3(0, 1) = T*cos(psi)*(w*cos(phi)*cos(theta) - u*sin(theta) + v*cos(theta)*sin(phi));
	block3(0, 2) = T*w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - T*v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - T*u*cos(theta)*sin(psi);

	block3(1, 0) = - T*v*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - T*w*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta));
	block3(1, 1) = T*sin(psi)*(w*cos(phi)*cos(theta) - u*sin(theta) + v*cos(theta)*sin(phi));
	block3(1, 2) = T*w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - T*v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + T*u*cos(psi)*cos(theta);

	block3(2, 0) = T*cos(theta)*(v*cos(phi) - w*sin(phi));
	block3(2, 1) = -T*(u*cos(theta) + w*cos(phi)*sin(theta) + v*sin(phi)*sin(theta));
	block3(2, 2) = 0;
 

	MatrixXf block4(6, 3);
	block4 = MatrixXf::Zero(6, 3);
	MatrixXf block5(6,6);
	block5 = MatrixXf::Identity(6,6);

	MatrixXf F(9, 9);
	F << block1, block2, block3, block4, block5;

	//cout << F << endl;

	return F;
 
}

MatrixXf vileMatriceD()
{
	MatrixXf D(9, 9);
	D = MatrixXf::Identity(9, 9);
	return T*D;
}

/*ROS EIGEN INTERFACE*/
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

