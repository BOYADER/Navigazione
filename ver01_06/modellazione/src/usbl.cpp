#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "modellazione/state_real.h"
#include "modellazione/usbl.h"
#include "math_utility.h"
#include "sensor_utility.h"
#include <cmath>
#include <random>



using namespace Eigen;
using namespace std;

geometry_msgs::Vector3 eta1, eta2;
modellazione::usbl usbl_measure; 

float dist; // distanza tra USBL e transponder usata per calcolare RTT

void usbl_state_read(const modellazione::state_real::ConstPtr& state)
{
  eta1 = state->eta_1;
  eta2 = state->eta_2;
}


void compute_measure(){
    Vector3f p_t(0, 0, 0); 
    Matrix3f J_inv = compute_jacobian1(eta2).transpose(); //ned to usbl(coincide con body)

    p_t = J_inv * ( p_t_ned - ( ros2eigen(eta1) + J_inv * (p_usbl) ) ); //J_inv * p_usbl = 0
    dist = p_t.norm();

    usbl_measure.pos.x = sqrt(p_t(0)*p_t(0) + p_t(1)*p_t(1) + p_t(2)*p_t(2));    			//range [m]
    usbl_measure.pos.y = atan2(p_t(1), p_t(0));                                  			//bearing [rad]
    usbl_measure.pos.z = atan2(p_t(2), sqrt(p_t(0)*p_t(0) + p_t(1)*p_t(1)));   				//elevation [rad]

    /*usbl_measure.pos.y = atan2(sin(usbl_measure.pos.y), cos(usbl_measure.pos.y));
    usbl_measure.pos.z = atan2(sin(usbl_measure.pos.z), cos(usbl_measure.pos.z));*/

    usbl_measure.counter++;

}


int compute_frequency(){
  return V_C/(2*dist);
}


int main(int argc, char **argv){

  ros::init(argc, argv, "usbl_sensor");

  ros::NodeHandle usbl_sensor;

  ros::Subscriber usbl_sub = usbl_sensor.subscribe("/state_real", 1, usbl_state_read);
  ros::Publisher usbl_pub = usbl_sensor.advertise<modellazione::usbl>("/sensor/usbl", 10);

  int RTT = 1;

  //Generazione rumore
  default_random_engine generator;
  normal_distribution<double> range_distribution(0, 1e-2);  		 		 //[m]
  normal_distribution<double> bearing_distribution(0, deg2rad(0.1));   		 //[rad]
  normal_distribution<double> elevation_distribution(0, deg2rad(0.1)); 		 //[rad]

  //ros::Rate loop_rate(RTT);  
  ros::spinOnce();  
  //loop_rate.sleep();                        

  while(ros::ok()){
  	ros::spinOnce();

  	RTT = compute_frequency(); 
    compute_measure();
    usbl_measure.pos.x += range_distribution(generator);
    usbl_measure.pos.y += bearing_distribution(generator);
    usbl_measure.pos.z += elevation_distribution(generator);

  	ros::Rate loop_rate(RTT);
    loop_rate.sleep();
    //ROS_INFO("sto per pubblicare: rbe = \n [%f \n %f \n %f] \n",usbl_measure.pos.x, usbl_measure.pos.y, usbl_measure.pos.z);
    usbl_pub.publish(usbl_measure);

  }


  return 0;
}

 