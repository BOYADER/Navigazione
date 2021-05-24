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
        p_body << x, y, z;
    }
   
    void set_data(geometry_msgs::Vector3 data_msg)
    {
        Rsb = MatrixXf::Identity(3, 3);
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
    Vector3f function(Vector3f pos, Vector3f rpy)
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
        elevation = acos(z/range);

        if( (x == 0) && (y == 0))  
            bearing = 0;
        else
            bearing = atan2(y, x);

        Vector3f usbl_corr(range, bearing, elevation);

        return usbl_corr;
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
        float term20, term21, term22, term23, term24, term25, term26, term27, term28;

        term1= x*cos(psi)*sin(phi) + y*sin(phi)*sin(psi) + y*cos(phi)*cos(psi)*sin(theta) - x*cos(phi)*sin(psi)*sin(theta);
        term18 = sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
        term20 = cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta);
        term21 = y*cos(theta)*sin(psi);
        term22 = cos(phi)*cos(theta)*(DEPTH - z);
        term23 = cos(theta)*sin(phi)*(DEPTH - z);
        term24 = x*cos(psi)*cos(theta);
        term25 = y*sin(psi)*cos(theta);
        term26 = sin(theta)*(DEPTH-z);
        term27 = cos(psi)*sin(phi);
        term28 = cos(psi)*sin(phi)*sin(theta);
        term19 = cos(phi)*sin(psi) - term28;
        term12 = x*term19 - y*term20 + term23;
        term13 = DEPTH*sin(theta) - z*sin(theta) + term24 + term21;
        term14 = pow(y*(term27 - sin(psi)*cos(phi)*sin(theta)) - x*(sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta)) + term22, 2);
        term15 = pow(x*(sin(psi)*cos(phi) - term28) - y*(cos(psi)*cos(phi) + sin(psi)*sin(phi)*sin(theta)) + term23, 2);
        term16 = pow(term26 + term25 + term24, 2);
        term17 = term27 - cos(phi)*sin(psi)*sin(theta);
        term10 = pow(DEPTH*sin(theta) - z*sin(theta) + term25 + term24 , 2);
        term11 = - x*term18 + y*term17 + term22;
        term2 = sqrt(1-(term14)/(term16 +  term14 +term15));
        term3 = sqrt(1-(term14)/(term10 +  term14 +term15)) * sqrt( term10 + term14 + term15 );
        term4 = 2*pow(term16 + term14 + term15, 3/2);
        term5 = term17 *term11*2 - term20*term12*2 + cos(theta)*sin(psi)*(term26 + term24 + term21)*2;
        term6 = - 2*term18*term11 + 2*term19*term12 + 2*cos(psi)*cos(theta)*(term26 + term24 + term21);
        term7 = term13*term13 + term12*term12;
        term8 = 2 * sqrt(term16 + term14 + term15);
        term9 = pow(term26 + term24 + term21, 2);

        if(term2 == 0)
            term2 = tolerance;

        if(term3 == 0)
            term3 = tolerance;
        
        if(term4 == 0)
            term4 = tolerance;

        if(term7 == 0)
            term7 = tolerance;

        if(term12 == 0)
            term12 =tolerance;

        if(term9 == 0)
            term9 =tolerance;

        if(term10 == 0)
            term10 =tolerance;

        if(term16 == 0)
            term16 =tolerance;

        if(term26 == 0)
            term26 = tolerance;

        if(term24 == 0)
            term24 =tolerance;
        
        if(term25 == 0)
            term25 = tolerance;

        if(term21 == 0)
            term21 = tolerance;

        H_usbl(0, 0) = term6/term8;
        H_usbl(0, 1) = term5/term8;
        H_usbl(0, 2) = -(DEPTH-z) / sqrt(term10 + term14 + term15);
        H_usbl(0, 3) = 0;
        H_usbl(0, 4) = 0;
        H_usbl(0, 5) = 0;
        H_usbl(0, 6) = 0;
        H_usbl(0, 7) = 0;
        H_usbl(0, 8) = 0;

        H_usbl(1, 0) = -(-DEPTH*cos(psi)*sin(phi) + y*cos(phi)*cos(theta) + z*cos(psi)*sin(phi) + DEPTH*cos(phi)*sin(psi)*sin(theta) - z*cos(phi)*sin(psi)*sin(theta))/term7;
        H_usbl(1, 1) = (DEPTH*sin(phi)*sin(psi) + x*cos(phi)*cos(theta) - z*sin(phi)*sin(psi) + DEPTH*cos(phi)*cos(psi)*sin(theta) - z*cos(phi)*cos(psi)*sin(theta))/term7;
        H_usbl(1, 2) = term1 / term7;
        H_usbl(1, 3) = 0;
        H_usbl(1, 4) = 0;
        H_usbl(1, 5) = 0;
        H_usbl(1,6) = - (term26 + term24 + term21) * term11 / (term12 * term12 + term9);
        H_usbl(1,7) = (sin(phi) - (term12 * (z * cos(theta) - DEPTH *  cos(theta) + x * cos(psi) * sin(theta) + y * sin(psi) * sin(theta))/term10) * term13 * term13) / term7;
        H_usbl(1,8) = -((x * term20 + y * term19) / (term26 + term24 + term21) - (y * cos(psi) * cos(theta) - x * cos(theta) * sin(psi) * term12) / (term16)) * term9 / (term12 * term12 + term9);

        H_usbl(2,0) = ((term18 / sqrt(term16 + term14 + term15))+(term6*term11/ term4))/term2 ;
        H_usbl(2,1) = -((term17 / sqrt(term16 + term14 + term15) )-(term5*term11 / term4))/term2 ;
        H_usbl(2,2) = -(( ((sin(theta)*(term26 + term24 + term21)*2 + cos(phi)*cos(theta)*term11*2 + cos(theta)*sin(phi)*term12*2)*term11)/ term4 )-( cos(phi)*cos(theta)/sqrt(term16 + term14 + term15) ))/term2 ;
        H_usbl(2,3) = 0 ;
        H_usbl(2,4) = 0 ;
        H_usbl(2,5) = 0 ;
        H_usbl(2,6) = (term12)/( term2*sqrt(term16 + term14 + term15) ) ;
        H_usbl(2,7) = cos(phi)*term13/term3 ;
        H_usbl(2,8) = term1/term3 ;
        
        return H_usbl;
    } 

    MatrixXf get_M()
    {
        Matrix3f block1;
        block1 = MatrixXf::Identity(3, 3);

        MatrixXf block2 = MatrixXf::Zero(3, 10);

        MatrixXf M_usbl(3, 13);
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

        MatrixXf M_dvl(3, 13);
        M_dvl << block1, block2, block3, SS(get_pbody());
        return M_dvl;
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
        MatrixXf M_depth(1, 13);
        M_depth << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
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

        MatrixXf M_ahrs(3, 13);
        M_ahrs << block1, block2, block3;
        
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
    //to do: get_DataLLA()

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
        Matrix3f block1;
        block1 = MatrixXf::Identity(3, 3);

        MatrixXf block2 = MatrixXf::Zero(3, 10);

        MatrixXf M_gps(3, 13);
        M_gps << block1, block2;
        return M_gps;
    }
};

