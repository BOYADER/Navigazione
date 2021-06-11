        /*_________________________________________*/
		/*_______________ LIBRARIES _______________*/

#include "math.h"
#include "/usr/include/eigen3/Eigen/Eigen"
using namespace std;
using namespace Eigen;

		/*_______________________________________________*/
		/*_______________ FUNCTIONS DECL. _______________*/
float rad2Deg(float);
float deg2Rad(float);

		/*_______________________________________________*/
		/*_______________ FUNCTIONS' BODY _______________*/

Vector3f lla2ned(Vector3f lla_local, Vector3f lla_0_local)
{
    Vector3f lla_rad;
    lla_rad     << deg2Rad(lla_local(0)),   deg2Rad(lla_local(1)),   0;

    Vector3f lla_0_rad;
    lla_0_rad   << deg2Rad(lla_0_local(0)), deg2Rad(lla_0_local(1)), 0;

    Vector3f ned_local  = Vector3f::Zero();

    double a = 6378137.0;
    double f = 1 / 298.257223563;
    double Rn, Rm;
    float dLat, dLon;

    dLat = lla_rad(0)-lla_0_rad(0);
    dLon = lla_rad(1)-lla_0_rad(1);

    Rn = a / sqrt( 1-(2*f-f*f)*sin(lla_0_rad(0))*sin(lla_0_rad(0)) );
    Rm = Rn*( 1-(2*f-f*f) )/( 1-(2*f-f*f)*sin(lla_0_rad(0))*sin(lla_0_rad(0)) );

    ned_local(0) = dLat / atan2(1, Rm);
    ned_local(1) = dLon / atan2(1, Rn*cos(lla_0_rad(0)));

    return ned_local;
}

Vector3f ned2lla(Vector3f eta1_local, Vector3f lla_0_local)
{
    Vector3f lla_local    = Vector3f::Zero();
    Vector3f lla_0_rad;
    lla_0_rad   << deg2Rad(lla_0_local(0)), deg2Rad(lla_0_local(1)), 0;

    double a = 6378137.0;
    double f = 1 / 298.257223563;
    double Rn, Rm;

    Rn = a / sqrt( 1-(2*f-f*f)*sin(lla_0_rad(0))*sin(lla_0_rad(0)) );
    Rm = Rn*( 1-(2*f-f*f) )/( 1-(2*f-f*f)*sin(lla_0_rad(0))*sin(lla_0_rad(0)) );

    lla_local(0) = rad2Deg( lla_0_rad(0) + atan2(1, Rm) * eta1_local(0) );
    lla_local(1) = rad2Deg( lla_0_rad(1) + atan2(1, Rn*cos(lla_0_rad(0))) * eta1_local(1) );
    lla_local(2) = 0;

    return lla_local;
}

float rad2Deg(float radians)
{
    return (radians / M_PI) * 180.0;
}

float deg2Rad(float degrees)
{
    return (degrees / 180.0) * M_PI;
}

