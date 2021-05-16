/*-----SENSOR UTILITY-----*/
#include <nav_pkg/math_utility.h> //Non serve, viene dichiarato dentro EKF

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "nav_pkg/Odom.h"
#include "nav_pkg/usbl.h"
#include "nav_pkg/ahrs.h"
#include "nav_pkg/imu.h"
#include "nav_pkg/dvl.h"
#include "nav_pkg/depth.h"
#include "nav_pkg/gps.h"

using namespace std;
using namespace Eigen;
//>Metodo per fornire la riga corrispondente al vettore. !!!!!!!
//>da ereditare per ogni sensore.
//>Inserire metodo per comunicare se si ha una nuova misura
//>Costruttore?
//>Inserire come oggetto il publisher?

/*class Sensor
{
    public:
    <sensor_type> sensor; //sensor data
    float sensor_id;

    virtual void sensor_callback() =0;
    <sensor_type> sensor_function(state_pred);

};*/

class USBL_sensor
{   
    public:
    geometry_msgs::Vector3 data;
    float id;

    Vector3f usbl_function(Vector3f eta1)
    {
        float x = eta1(0);
        float y = eta1(1);
        float z = eta1(2);

        float range = sqrt(x*x + y*y + (z-DEPTH)*(z-DEPTH));
        float bearing = atan2(y, x);
        float elevation = acos((z-DEPTH)/range);

        Vector3f usbl_corr(range, bearing, elevation);

        return usbl_corr;
    }

    MatrixXf get_H(Vector3f eta1)
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

        MatrixXf H_usbl(3, 9);
        H_usbl << block_usbl, block1;
        return H_usbl;
    }

    /*void usbl_callback(const nav_pkg::usbl::ConstPtr& msg)
    {
        usbl_data = msg->pos;
        usbl_id = msg->counter;
        ROS_WARN("USBL DATA: \n");
        ROS_INFO("Posizione: [%f, %f]\n", msg->pos.x, msg->pos.y);
    }*/

};

class DVL_sensor
{   
    public:
    geometry_msgs::Vector3 data;
    float id;

    Vector3f dvl_function(Vector3f ni1, Vector3f ni2)
    {
        Matrix3f Rb_dvl = MatrixXf::Identity(3,3); //per ora e' identita'
        Vector3f p_dvl(0, 0, 0);

        Vector3f corr;
        corr = Rb_dvl * (ni1 + SS(ni2)*p_dvl);

        return corr;
    }

    /*To do: getH*/

    /*void usbl_callback(const nav_pkg::usbl::ConstPtr& msg)
    {
        usbl_data = msg->pos;
        usbl_id = msg->counter;
        ROS_WARN("USBL DATA: \n");
        ROS_INFO("Posizione: [%f, %f]\n", msg->pos.x, msg->pos.y);
    }*/

};

class DEPTH_sensor
{
    public:
    std_msgs::Float64 data;
    float id;

    float depth_function(Vector3f eta1)
    {
        float corr = eta1.z();
        return corr;
    }

};

class GPS_sensor
{
    public:
    geometry_msgs::Vector3 data;
    float id;
    bool under_water;

    Vector3f gps_function(Vector3f eta1) //da modificare: restituisce lat e long
    {
        float x = eta1(0);
        float y = eta1(1);
        float z = eta1(2);

        Vector3f posit(x, y, z);
        return posit;
    } 

};


