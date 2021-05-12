#include <ostream>
#include <iostream>
#include </home/daniele/eigen/Eigen/Eigen>
#include "ros/ros.h"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "prova_eigen");
	ros::NodeHandle node_obj;

	while(ros::ok())
	{
		ROS_INFO("TEST EIGEN\n");
		MatrixXd test = MatrixXd::Zero(10,10);
		cout << test << endl;
	}


	return 0;
}
