#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "nav_pkg/Odom.h"
#include "nav_pkg/nav_error.h"
#include "modellazione/state_real.h"
#include <nav_pkg/math_utility.h>
#include <nav_pkg/geolib.h>

using namespace std;
using namespace Eigen;
  
#define LAT_0   45.110735     
#define LONG_0  7.640827     
#define ALT_0   0              

/*Variabili globali*/
//Stato
geometry_msgs::Vector3 eta1;
geometry_msgs::Vector3 ni1;
geometry_msgs::Vector3 eta2;
//Stima
geometry_msgs::Vector3 eta1_cap;
geometry_msgs::Vector3 ni1_cap;
geometry_msgs::Vector3 eta2_cap;

Vector3f ned_lla_0(LAT_0, LONG_0, ALT_0);

void print_info(VectorXf error);
float get_MSE(float mse_curr);

void state_callback(const modellazione::state_real::ConstPtr& msg)
{
    eta1 = msg->eta_1;
    ni1 = msg->ni_1;
    eta2 = msg->eta_2;
    //ROS_INFO("Stato vero\n");
}

void odom_callback(const nav_pkg::Odom::ConstPtr& msg)
{
    //eta1_cap = eigen2ros(lla2ned(ros2eigen(msg->lld), ned_lla_0));
    eta1_cap = msg->lld;
    ni1_cap = msg->lin_vel;
    eta2_cap = msg->rpy;
    /*ROS_INFO("Stato stimato\n");
    cout << eta1_cap << endl;*/

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "performance_analysis");
	ros::NodeHandle node_obj;
	
	ros::Publisher pub = node_obj.advertise<nav_pkg::nav_error>("/error",10);
	ros::Subscriber sub_state=node_obj.subscribe("/state_real", 1, state_callback);
	ros::Subscriber sub_filter=node_obj.subscribe("/odom", 1, odom_callback);
    ros::Rate loop_rate(10);
    
    while(ros::ok())
    {
        ros::spinOnce();
        
        VectorXf state_real(9);
        state_real << ros2eigen(eta1), ros2eigen(ni1), ros2eigen(eta2);

        VectorXf state_cap(9);
        state_cap << ros2eigen(eta1_cap), ros2eigen(ni1_cap), ros2eigen(eta2_cap);

        VectorXf error = state_real - state_cap;

        print_info(error);

        Vector3f err_pos(error(0), error(1), error(2));
        Vector3f err_vel(error(3), error(4), error(5));
        Vector3f err_rpy(error(6), error(7), error(8));

        /*if(err_pos(0) > 0.1)
            ROS_WARN("picco pos!\n");*/

        nav_pkg::nav_error err_msg;
        err_msg.pos = eigen2ros(err_pos);
        err_msg.rpy = eigen2ros(err_rpy);
        err_msg.lin_vel = eigen2ros(err_vel);

        float MSE = error.squaredNorm();
        /*float MSE_pos = err_pos.squaredNorm();
        float MSE_vel = err_vel.squaredNorm();
        float MSE_rpy = err_rpy.squaredNorm();*/

        err_msg.MSE = get_MSE(MSE);
        /*err_msg.MSE_pos = get_MSE(MSE_pos);
        err_msg.MSE = get_MSE(MSE);
        err_msg.MSE = get_MSE(MSE);*/

        pub.publish(err_msg);
        loop_rate.sleep();

    }

    return 0;
}

void print_info(VectorXf error)
{
    //ROS_WARN("Performance Analysis: \n");
    cout << error << endl;
}

float get_MSE(float mse_curr)
{
    static int n = 0;
    static float mean = 0.0;

    ++n;
    float delta = mse_curr -mean;
    mean += delta/n;

    return mean;
}