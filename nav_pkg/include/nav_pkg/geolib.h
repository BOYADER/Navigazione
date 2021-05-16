#include "math.h"
#include <eigen/Eigen/Dense>

using namespace std;
using namespace Eigen;
 
/*------Parameters Declarations------*/
static const double a       = 6378137;
static const double b       = 6356752.3142;
static const double esq     = 6.69437999014 * 0.001;
static const double e1sq    = 6.73949674228 * 0.001;
static const double f       = 1 / 298.257223563;

/*------Functions Declarations------*/
float rad2Deg(float);
float deg2Rad(float);
Matrix3f _nRe_( Vector3f);
float phiP(Vector3f ned_ecef_0);

/*------Functions Body------*/

/*------In: v2:vect in lla, Out: vect in ecef------*/
Vector3f geodetic2Ecef( Vector3f v2 )
{
    Vector3f v1; 

    float lat_rad = deg2Rad( v2(0) );
    float lon_rad = deg2Rad( v2(1) );
    float xi = sqrt(1 - esq * sin( lat_rad ) * sin( lat_rad ) );
    v1(0) = ( a / xi + v2(2) ) * cos( lat_rad ) * cos( lon_rad );
    v1(1) = ( a / xi + v2(2) ) * cos( lat_rad ) * sin( lon_rad );
    v1(2) = ( a / xi * ( 1 - esq ) + v2(2) ) * sin( lat_rad );
    return v1;
 }

/*------In: v2:vect in ecef, Out: vect in lla------*/     
Vector3f ecef2Geodetic( Vector3f v2)
{
    float x = v2(0);
    float y = v2(1);
    float z = v2(2);

    Vector3f v1;

    float r = sqrt(x * x + y * y);
    float Esq = a * a - b * b;
    float F = 54 * b * b * z * z;
    float G = r * r + (1 - esq) * z * z - esq * Esq;
    float C = (esq * esq * F * r * r) / pow(G, 3);

    float S = pow(1 + C + sqrt(C * C + 2 * C), 1 / 3);
    float P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
    float Q = sqrt(1 + 2 * esq * esq * P);
    float r_0 = -(P * esq * r) / (1 + Q) + sqrt(0.5 * a * a * (1 + 1.0 / Q) - P * (1 - esq) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);

    float U = sqrt(pow((r - esq * r_0), 2) + z * z);
    float V = sqrt(pow((r - esq * r_0), 2) + (1 - esq) * z * z);
    float Z_0 = b * b * z / (a * V);

    v1(0) = rad2Deg(atan((z + e1sq * Z_0) / r));
    v1(1) = rad2Deg(atan2(y, x));
    v1(2) = U * (1 - b * b / (a * V));

    return v1;
}

/*------In: v2: vect ned, v0: coords ECEF of NED origin, Out: vect in ecef------*/
Vector3f ned2Ecef( Vector3f v2, Vector3f v0 )
{
    Vector3f ned, ret, v1;
    ned(0) = v2(0);
    ned(1) = v2(1);
    ned(2) = -v2(2);

    Vector3f v0_lla = ecef2Geodetic(v0);
    Matrix3f _ned_to_ecef_matrix = _nRe_(v0_lla).transpose();
    ret = _ned_to_ecef_matrix * ned;

    v1(0) = ret(0) + v0(0);
    v1(1) = ret(1) + v0(1);
    v1(2) = ret(2) + v0(2);

    return v1;
}

/*------In: v2: vect ecef, v0: coords ECEF of NED origin, Out: vect in ned------*/
Vector3f ecef2Ned(Vector3f v2_ecef, Vector3f ned_ecef_0)
{
    Vector3f vect, ret, v;
    vect(0) = v2_ecef(0) - ned_ecef_0(0);
    vect(1) = v2_ecef(1) - ned_ecef_0(1);
    vect(2) = v2_ecef(2) - ned_ecef_0(2);

    Vector3f vv;
    vv(0) = phiP(ned_ecef_0); 
    vv(1) = ecef2Geodetic(ned_ecef_0)(1);
    vv(2) = 0;

    Matrix3f _ecef_to_ned_matrix = _nRe_(vv); //this conversion needs phiP and longitude
    ret = _ecef_to_ned_matrix * vect;
    v(0) = ret(0);
    v(1) = ret(1);
    v(2) = -ret(2);
    return v;
}



/*------In: v2: vect ned, v0: coords ECEF of NED origin, Out: vect in lla------*/
Vector3f ned2Geodetic(Vector3f v2, Vector3f v0)
{
    Vector3f v1_ecef = ned2Ecef(v2, v0);
    Vector3f v_geo = ecef2Geodetic(v1_ecef);
    return v_geo;
}

/*------In: lla: vect lla, ned_lla_0: coords GEOD of NED origin, Out: vect in ned------*/
Vector3f geodetic2Ned(Vector3f lla, Vector3f ned_lla_0)
{
    Vector3f ned_ecef_0 = geodetic2Ecef(ned_lla_0); 
    Vector3f v_ecef = geodetic2Ecef(lla);
    Vector3f v_ned = ecef2Ned(v_ecef, ned_ecef_0);
    return v_ned;
}

/*------Rotational matrix ECEF->NED, In: v2: vect lla. Usefull for coords changes  ------*/
Matrix3f _nRe_( Vector3f v2) 
{
    float lat_rad = deg2Rad(v2(0));
    float lon_rad = deg2Rad(v2(1));
    
    float sLat = sin(lat_rad);
    float sLon = sin(lon_rad);
    float cLat = cos(lat_rad);
    float cLon = cos(lon_rad);

    Matrix3f ret;
    ret(0, 0) = -sLat * cLon;
    ret(0, 1) = -sLat * sLon;
    ret(0, 2) = cLat;
    ret(1, 0) = -sLon;
    ret(1, 1) = cLon;
    ret(1, 2) = 0.0;
    ret(2, 0) = cLat * cLon;
    ret(2, 1) = cLat * sLon;
    ret(2, 2) = sLat;

    return ret;
}

/*------ Conversion rad->deg and viceversa------*/
float rad2Deg(float radians)
{
    return (radians / M_PI) * 180.0;
}

float deg2Rad(float degrees)
{
    return (degrees / 180.0) * M_PI;
}

/*------Coefficient for ecef to ned conversion, In: v0: coords ECEF of NED origin------*/
float phiP(Vector3f ned_ecef_0)
{
    float p = atan2(ned_ecef_0(2), sqrt(pow(ned_ecef_0(0), 2) + pow(ned_ecef_0(1), 2)));
    return p;
}



