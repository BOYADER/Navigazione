// Prova funzioni di Costy-Costy

#include <ros/ros.h>
#include <eigen/Eigen/Eigen>
#include <nav_pkg/geolib.h>
#include <nav_pkg/geolib_old.h>
#include <nav_pkg/sensor_class.h>

#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "nav_pkg/usbl.h"
#include "nav_pkg/ahrs.h"
#include "nav_pkg/imu.h"
#include "nav_pkg/dvl.h"
#include "nav_pkg/depth.h"
#include "nav_pkg/gps.h"
#include "nav_pkg/Odom.h"

using namespace std;
using namespace Eigen;

#define DEPTH 	50 			//profondità fondale
#define LAT_0 	45.110735 	//latitudine orig. 	{NED} 
#define LONG_0 	7.640827 	//longitudine orig. {NED}
#define ALT_0 	0			//altitudine orig. 	{NED}

int main()
{
    Vector3f ned_lla_0(LAT_0, LONG_0, ALT_0);
    Vector3f ned_AUV( 100, 0, 0 );
    Vector3f ned_AUV_deriv;

    cout << "\n\nInput Auv pos:\t\t" << ned_AUV(0) << "\t" << ned_AUV(1) << "\t" << ned_AUV(2) << endl;
    cout << "Input LLA 0:\t\t" << ned_lla_0(0) << "\t" << ned_lla_0(1) << "\t" << ned_lla_0(2)<< "\n" << endl;

    Vector3f lla_AUV_Matlab, lla_AUV_Geodesy;

    lla_AUV_Geodesy << ned2Geodetic(ned_AUV, geodetic2Ecef(ned_lla_0));
    cout << "LLA_Geodesy Auv:\t" << lla_AUV_Geodesy(0) << "\t" << lla_AUV_Geodesy(1) << "\t" << lla_AUV_Geodesy(2) << endl;

    lla_AUV_Matlab  << ned2lla(ned_AUV, ned_lla_0);
    cout << "LLA_Matlab Auv:\t\t" << lla_AUV_Matlab(0) << "\t" << lla_AUV_Matlab(1) << "\t" << lla_AUV_Matlab(2) << "\n" << endl;

    ned_AUV_deriv   << lla2ned(lla_AUV_Geodesy, ned_lla_0);
    cout << "NED_Matlab Auv:\t" << ned_AUV_deriv(0) << "\t" << ned_AUV_deriv(1) << "\t" << ned_AUV_deriv(2) << endl;

    ned_AUV_deriv   << geodetic2Ned(lla_AUV_Matlab, ned_lla_0);
    cout << "NED_Geodesy Auv:\t" << ned_AUV_deriv(0) << "\t" << ned_AUV_deriv(1) << "\t" << ned_AUV_deriv(2) << "\n\n\n" << endl;

    return 0;
}
