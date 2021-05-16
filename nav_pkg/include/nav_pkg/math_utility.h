/*----------------MATH UTILITY----------------*/
#include <ostream>
#include <iostream>
#include <eigen/Eigen/Eigen>

using namespace std;
using namespace Eigen;

/*------------PARAMETERS DEFINITIONS------------*/
#define T 0.1
#define DEPTH 50 //profondità fondale
#define LAT_0 45.110735 //latitudine orig. {NED} 
#define LONG_0 7.640827 //longitudine orig. {NED}
#define ALT_0 0			//altitudine orig. {NED}
#define n 9 //num. state variable

/*------------CUSTOM FUNCTION DECLARATION------------*/
//clear matrix 
float tolerance = 1*pow(10, -5);
MatrixXf clear_small_number(MatrixXf matrix, float tol);

//Jacobian and Skew Symmetrix matrix function
Matrix3f Jacobian_RPY(Vector3f rpy);
Matrix3f SS(Vector3f v);

//Linearization Matrix
MatrixXf vileMatriceF(Vector3f rpy);
MatrixXf vileMatriceD(Vector3f ni1, Vector3f rpy);
MatrixXf vileMatriceH(Vector3f eta1, Vector3f ni1, Vector3f ni2);

/*----------FUNCTION--------*/
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

/*VILI MATRICI*/
MatrixXf vileMatriceF(Vector3f rpy)
{
	/*F=[I	0	T*J1]
		[0	I	  0	]
		[0	0	  I	]*/
	

	MatrixXf block1(3, 3);
	block1 = MatrixXf::Identity(3, 3);
	MatrixXf block2(3, 3);
	block2 = T*Jacobian_RPY(rpy);
	MatrixXf block3(3, 3);
	block3 = MatrixXf::Zero(3, 3);
	MatrixXf block4(6, 3);
	block4 = MatrixXf::Zero(6, 3);
	MatrixXf block5(6,6);
	block5 = MatrixXf::Identity(6,6);

	MatrixXf F(9, 9);
	F << block1, block2, block3, block4, block5;

	//cout << F << endl;

	return F;
 
}

MatrixXf vileMatriceD(Vector3f ni1, Vector3f rpy)
{
	float phi = rpy(0);
	float theta = rpy(1);
	float psi = rpy(2);

	float u = ni1(0);
	float v = ni1(1);
	float w = ni1(2);

	MatrixXf block(3, 3); 

	block(0, 0) = T*v*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + T*w*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta));
	block(0, 1) = T*cos(psi)*(w*cos(phi)*cos(theta) - u*sin(theta) + v*cos(theta)*sin(phi));
	block(0, 2) = T*w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - T*v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - T*u*cos(theta)*sin(psi);

	block(1, 0) = - T*v*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - T*w*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta));
	block(1, 1) = T*sin(psi)*(w*cos(phi)*cos(theta) - u*sin(theta) + v*cos(theta)*sin(phi));
	block(1, 2) = T*w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - T*v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + T*u*cos(psi)*cos(theta);

	block(2, 0) = T*cos(theta)*(v*cos(phi) - w*sin(phi));
	block(2, 1) = -T*(u*cos(theta) + w*cos(phi)*sin(theta) + v*sin(phi)*sin(theta));
	block(2, 2) = 0;

	MatrixXf block1(3, 6);
	block1 = MatrixXf::Zero(3, 6);

	MatrixXf block2(6,6);
	block2 = MatrixXf::Identity(6,6);

	MatrixXf D(9, 9);
	D << block, block1, block1.transpose(), block2;
	return D;
 
}

//->Hard code: Generalizza Rdvl e pdvl
MatrixXf vileMatriceH(Vector3f eta1, Vector3f ni1, Vector3f ni2)
{
	float x = eta1(0);
	float y = eta1(1);
	float z = eta1(2);

	Matrix3f block_usbl;
	float modulo = sqrt(x*x + y*y + (z-DEPTH)*(z-DEPTH));
	block_usbl(0, 0) = x/modulo;
	block_usbl(0, 1) = y/modulo;
	block_usbl(0, 2) = (z-DEPTH)/modulo;

	float modulo2 = sqrt(x*x + y*y);
	block_usbl(1, 0) = -y/modulo2;
	block_usbl(1, 1) = x/modulo2;
	block_usbl(1, 2) = 0;

	float modulo3 = (x*x + y*y + (z-DEPTH)*(z-DEPTH)) * modulo2;
	block_usbl(2, 0) = x*(z-DEPTH)/modulo3;
	block_usbl(2, 1) = y*(z-DEPTH)/modulo3;
	block_usbl(2, 2) = -modulo2/(x*x + y*y + (z-DEPTH)*(z-DEPTH));

	MatrixXf block1(3, 6);
	block1 = MatrixXf::Zero(3, 6);

	Matrix3f block2 = MatrixXf::Zero(3, 3);

	Matrix3f block3 = MatrixXf::Identity(3, 3);

	float p_dvl = 3/10;
	Matrix3f block_dvl;
	block_dvl << 0, 0.5*p_dvl, 0, -0.5*p_dvl, 0, p_dvl, 0, -p_dvl, 0;

	VectorXf block_depth(9);
	block_depth << 0, 0, 1, 0, 0, 0, 0, 0, 0;

	MatrixXf H(7,9);
	H << block_usbl, block1, block2, block3, block_dvl, block_depth.transpose();

	return H;
}



