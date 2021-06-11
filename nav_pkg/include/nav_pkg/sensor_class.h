#include <nav_pkg/math_utility.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "nav_pkg/Odom.h"
#include "nav_pkg/usbl.h"
#include "nav_pkg/ahrs.h"
#include "nav_pkg/dvl.h"
#include "nav_pkg/depth.h"
#include "nav_pkg/gps.h"

#define DEPTH 50 
#define usbl_tol 0.002

using namespace std;
using namespace Eigen;

class Sensor
{
    Matrix3f Rsb; 
    Vector3f p_body;

    float id;
    geometry_msgs::Vector3 data;

    bool isNew;

    public:
    Sensor(Matrix3f R = MatrixXf::Identity(3, 3), Vector3f p_vector = Vector3f::Zero())
    {
        Rsb = R;
        p_body = p_vector;
    }

    Sensor(Matrix3f R)
    {
        Rsb = R;
        p_body = Vector3f::Zero();
    }

    Sensor(Vector3f p_vector)
    {
        Rsb = MatrixXf::Identity(3, 3);
        p_body = p_vector;
    }

    Sensor(float x, float y, float z)
    {
        Rsb = MatrixXf::Identity(3, 3);
        p_body << x, y, z;
    }
   
    void set_data(geometry_msgs::Vector3 data_msg)
    {
        data = data_msg;
    }

    void set_data(float x, float y, float z)
    {
        data.x =x;
        data.y =y;
        data.z =z;
    }

    void set_id(float id_msg)
    {
        id = id_msg;
    }

    geometry_msgs::Vector3 get_data()
    {
        return data;
    }

    float get_id()
    {
        return id;
    }

    Vector3f get_pbody()
    {
        return p_body;
    }

    void set_pbody(Vector3f p_sens)
    {
        p_body = p_sens;
    }

    void set_pbody(float p1, float p2, float p3)
    {
        p_body << p1, p2, p3;
    }

    Matrix3f get_Rsb()
    {
        return Rsb;
    }

    void set_Rsb(Matrix3f R)
    {
        Rsb = R;
    }

    void set_isNew()
    {
        isNew = true;
    }

    void set_isOld()
    {
        isNew = false;
        //ROS_INFO("Misura usata\n");
    }

    bool get_isNew()
    {
        return isNew;
    }
};

//Classi figlie
class Usbl:public Sensor
{  
    public:
    //using Sensor::Sensor; //eredito i costruttori da Sensor
    Usbl()
    {
        set_pbody(0, 0, DEPTH);
    }

    //ridefinizione delle funzioni
    VectorXf function(Vector3f pos, Vector3f rpy)
    {
        float range;
        float bearing;
        float elevation;

        Matrix3f J1 = Jacobian_RPY(rpy);
        Vector3f eta_b = J1.transpose() * (get_pbody() - pos);
        
        float x = eta_b(0);
        float y = eta_b(1);
        float z = eta_b(2);

        range = sqrt(x*x + y*y + z*z);
        elevation = atan2(z, sqrt(x*x + y*y));

        //Punto di Singolarità: x == 0, y == 0 -> elevation == M_PI/2
       
        if((abs(elevation) > M_PI_2 - usbl_tol) &&  (abs(elevation) < M_PI_2 + usbl_tol)) //~ M_PI/2
        {
            //ROS_WARN("USBL: Singolarita'! \n");
            Vector2f usbl_corr(range, elevation);
            return usbl_corr;
        }

        else
        {
            bearing = atan2(y, x);
            Vector3f usbl_corr(range, bearing, elevation);
            return usbl_corr;
        }

    }

    MatrixXf get_H(Vector3f pos, Vector3f rpy)
    {
        float x = pos(0);
        float y = pos(1);
        float z = pos(2);

        float phi = rpy(0);
        float theta = rpy(1);
        float psi = rpy(2);

        MatrixXf H_usbl(3, 9);

        //Usiamo pretty(H) per individuare i termini ricorrenti.
        float term1, term2, term3, term4, term5, term6, term7, term8, term9;
        float term10, term11, term12, term13, term14, term15, term16, term17, term18, term19;
        float term20, term21, term22, term23, term24, term25, term26, term27, term28, term29;

        //Indip. da altri termini
        term1 = y*cos(psi)*cos(theta) - x*cos(theta)*sin(psi);
        term2 = y*sin(psi)*sin(theta);
        term3 = x*cos(psi)*sin(theta);
        term19 = cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta);
        term21 = sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
        term22 = y*cos(theta)*sin(psi);
        term23 = cos(phi)*cos(theta)*(DEPTH - z);
        term24 = cos(theta)*sin(phi)*(DEPTH - z);
        term25 = x*cos(psi)*cos(theta);
        term26 = y*sin(psi)*cos(theta);
        term27 = sin(theta)*(DEPTH - z);
        term28 = cos(psi)*sin(phi)*sin(theta);
        term29 = cos(psi)*sin(phi);

        //da altri termini
        term18 = cos(phi)*sin(psi) - term28;
        term20 = term29 - cos(phi)*sin(psi)*sin(theta);
        term14 = powf(term27 + term25 + term22, 2.0);
        term15 = powf(y*(term29 - sin(psi)*cos(phi)*sin(theta)) - x*(sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta)) + term23, 2.0);
        term16 = powf(x*(sin(psi)*cos(phi) - term28) - y*(cos(psi)*cos(phi) + sin(psi)*sin(phi)*sin(theta)) + term24, 2.0);
        term17 = powf(term27 + term26 + term25, 2.0);
        term9 = cos(theta)*sin(psi)*(term27 + term25 + term22)*2.0;
        term10 = cos(psi)*cos(theta)*(term27 + term25 + term22)*2.0;
        term11 = powf(term17 + term16, 1.5);
        term12 = x*term18 - y*term19 + term24;
        term13 = - x*term21 + y*term20 + term23;
        term4 = term14 + powf(term13, 2.0)  + powf(term12, 2.0);
        term5 = 2.0*sqrt(term17 + term15 + term16);
        term6 = x*term19 + y*term18;
        term7 = powf(DEPTH*sin(theta) - z*sin(theta) + term26 + term25, 2.0);
        term8 = powf(DEPTH*sin(theta) - z*sin(theta) + term25 + term22, 2.0);

        H_usbl(0, 0) = (- term21*term13*2.0 + term18*term12*2.0 + term10)/term5;
        H_usbl(0, 1) = (term20*term13*2.0 - term19*term12*2.0 + term9)/term5;
        H_usbl(0, 2) = -(DEPTH-z)/(sqrt(term7 + term15 + term16));
        H_usbl(0, 3) = 0;
        H_usbl(0, 4) = 0;
        H_usbl(0, 5) = 0;
        H_usbl(0, 6) = 0;
        H_usbl(0, 7) = 0;
        H_usbl(0, 8) = 0;

        H_usbl(1, 0) = -((- DEPTH*cos(psi)*sin(phi) + y*cos(phi)*cos(theta) + z*cos(psi)*sin(phi) + DEPTH*cos(phi)*sin(psi)*sin(theta) - z*cos(phi)*sin(psi)*sin(theta))/(term12*term12 + term8));
        H_usbl(1, 1) = (DEPTH*sin(phi)*sin(psi) + x*cos(phi)*cos(theta) - z*sin(phi)*sin(psi) + DEPTH*cos(phi)*cos(psi)*sin(theta) - z*cos(phi)*cos(psi)*sin(theta))/(term12*term12 + term8);
        H_usbl(1, 2) = (x*cos(psi)*sin(phi) + y*sin(phi)*sin(psi) + y*cos(phi)*cos(psi)*sin(theta) - x*cos(phi)*sin(psi)*sin(theta))/(term12*term12 + term8);
        H_usbl(1, 3) = 0;
        H_usbl(1, 4) = 0;
        H_usbl(1, 5) = 0;
        H_usbl(1, 6) = -(term13*(term27+term25+term22))/(term12*term12 + term14);
        H_usbl(1, 7) = ((sin(phi) - (term12*(z*cos(theta) - DEPTH*cos(theta) + term3 + term2))/(term7))*term8)/(term12*term12 +term8);
        H_usbl(1, 8) = -((term6)/(term27 + term25 + term22) - (term1*term12)/(term17))*term14/(term12*term12 + term14);

        H_usbl(2, 0) = -(((term21)/(sqrt(term17+term16)) + (term18*term12*2.0 + term10)*(term13)/(2.0*term11))*(term12*term12 + term14)/(term4));
        H_usbl(2, 1) = (((term20)/(sqrt(term17+term16)) + (term19*term12*2.0 - term9)*(term13)/(2.0*term11))*(term12*term12 + term14)/(term4));
        H_usbl(2, 2) = -((((cos(phi)*cos(theta))/(sqrt(term17+term16))) - ((sin(theta)*(term27+term25+term22)*2.0 + cos(theta)*sin(phi)*term12 *2.0)*term13/(2.0*term11)))*(term12*term12+term14))/term4;
        H_usbl(2,3) = 0;
        H_usbl(2,4) = 0;
        H_usbl(2,5) = 0;
        H_usbl(2,6) = - ((term12/sqrt(term17+term16)) + (term13 * term13 * term12 / term11)) * (term12 * term12 + term14) / term4;
        H_usbl(2,7) = - ((((cos(phi) * sin(theta) * (DEPTH - z) + x * cos(phi) * cos(psi) * cos(theta) + y * cos(phi) * cos(theta) * sin(psi))/ sqrt(term17 + term16)) - (((term12 * (sin(phi) * sin(theta) * (DEPTH - z) + x * cos(psi) * cos(theta) * sin(phi) + y * cos(theta) * sin(phi) * sin(psi)) * 2) + (term27 + term25 + term22) * (term3 - cos(theta) * (DEPTH - z) + term2) *2) * term13 / (2 * term11))) * (term12 * term12 + term14)) / term4;			        
        H_usbl(2, 8) = -((x*term20 + y*term21)/(sqrt(term17 + term16)) + (term1*(term27 + term25 + term22)*2.0 + 2.0*term6*term12)*(term13)/(2.0*term11))*(term12*term12+term14)/(term4);


        return H_usbl;
    } 

    MatrixXf get_M()
    {
        Matrix3f block1;
        block1 = MatrixXf::Identity(3, 3);

        MatrixXf block2 = MatrixXf::Zero(3, 13);

        MatrixXf M_usbl(3, 16);
        M_usbl << block1, block2;
        return M_usbl;
    }

};

class Dvl:public Sensor
{   
    public:
    using Sensor::Sensor;

    Vector3f function(Vector3f ni1)
    {
        Vector3f dvl_fun = get_Rsb()*ni1;
        return dvl_fun;
    }

    MatrixXf get_H()
    {
        Matrix3f block1;
        block1 = MatrixXf::Zero(3, 3);

        Matrix3f block2;
        block2 = MatrixXf::Identity(3, 3);

        Matrix3f block3;
        block3 = MatrixXf::Zero(3, 3);
        
        MatrixXf H_dvl(3, 9);
        H_dvl << block1, block2, block3;
        
        return H_dvl;
    }

    MatrixXf get_M()
    {
        Matrix3f block1;
        block1 = MatrixXf::Zero(3, 3);

        Matrix3f block2;
        block2 = MatrixXf::Identity(3, 3);

        MatrixXf block3 = MatrixXf::Zero(3, 4);

        MatrixXf M_dvl(3, 16);
        M_dvl << block1, block2, block3, SS(get_pbody()), MatrixXf::Zero(3, 3);
        return M_dvl;
    }

    Vector3f function_PF(Vector3f ni1, Vector3f ni2)
    {
        Vector3f dvl_fun_pf = get_Rsb()*(ni1 + SS(ni2)*get_pbody());
        return dvl_fun_pf;
    }
};

class Depth: public Sensor
{
    public:
    using Sensor::Sensor;

    void set_data(float z)
    {
        Sensor::set_data(0, 0, z);
    }

    float get_data()
    {
        return Sensor::get_data().z;
    }

    float function(Vector3f pos)
    {
        float depth = pos.z();
        return depth;
    }

    MatrixXf get_H()
    {
        MatrixXf H_depth(1, 9);
        H_depth << 0, 0, 1, 0, 0, 0, 0, 0, 0;
        return H_depth;
    }

    MatrixXf get_M()
    {
        MatrixXf M_depth(1, 16);
        M_depth << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        return M_depth;
    }

};

class Ahrs: public Sensor
{
    public:
    using Sensor::Sensor;

    Vector3f function(Vector3f eta2)
    {
        return eta2;
    }
    
    MatrixXf get_H()
    {
        MatrixXf H_ahrs(3, 9);

        MatrixXf block1(3,6);
        block1 = MatrixXf::Zero(3, 6);

        Matrix3f block2;
        block2 = MatrixXf::Identity(3, 3);

        H_ahrs << block1, block2;
        
        return H_ahrs;
    }

    MatrixXf get_M()
    {
        MatrixXf block1(3, 7);
        block1 = MatrixXf::Zero(3, 7);

        Matrix3f block2;
        block2 = MatrixXf::Identity(3, 3);

        Matrix3f block3;
        block3 = MatrixXf::Zero(3, 3);

        MatrixXf M_ahrs(3, 16);
        M_ahrs << block1, block2, block3, MatrixXf::Zero(3, 3);
        
        return M_ahrs;
    }

    

};

class Gps: public Sensor
{
    bool under_water;

    public:
    using Sensor::Sensor;

    void set_underWater(bool flag)
    {
        under_water = flag;
    }

    bool get_underWater()
    {
        return under_water;
    }

    Vector3f function(Vector3f eta1)
    {
        return eta1;
    }

    MatrixXf get_H()
    {
        MatrixXf H_gps(3, 9);

        Matrix3f block1;
        block1 = MatrixXf::Identity(3, 3);

        MatrixXf block2(3,6);
        block2 = MatrixXf::Zero(3, 6);

        H_gps << block1, block2;
        
        return H_gps;
    }

    MatrixXf get_M()
    {
        Matrix3f block2;
        block2 = MatrixXf::Identity(3, 3);

        MatrixXf block1 = MatrixXf::Zero(3, 13);

        MatrixXf M_gps(3, 16);
        M_gps << block1, block2;
        return M_gps;
    }
};

