#include <iostream>
#include "/usr/include/eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include "modellazione/state_real.h"
#include "modellazione/tau.h"
#include <sstream>
#include "constant.h"
#include <math.h>
#include "math_utility.h"
#include "nav_pkg/Odom.h"

bool lets_go=false;


//QUESTO NODO DEVE SIMULARE LA PUBBLICAZIONE SUL TOPIC "tau"
//DEL MSG tau.msg (geometry_msgs/Wrench tau)

modellazione::tau control_tau;
void init_tau (){
control_tau.tau.force.x = 0; // 10*sin(0.5*ros::Time::now().toSec());
control_tau.tau.force.y = 0; //cos(ros::Time::now().toSec());// frand(-5,5);
control_tau.tau.force.z =  0;// frand(-5,5);
control_tau.tau.torque.x = 0;//frand(-5,5);
control_tau.tau.torque.y = 0;//frand(-5,5);
control_tau.tau.torque.z = 0;//frand(-5,5);
}

void odom_callback(const nav_pkg::Odom::ConstPtr& msg)
{
  lets_go = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_control");

  ros::NodeHandle control;

  ros::Publisher control_pub = control.advertise<modellazione::tau>("tau", MAX_QUEUE_LENGTH);
  ros::Subscriber sub_odom = control.subscribe("/odom", 1, odom_callback);

  ros::Rate loop_rate(5);
  int count = 0;
  int i=0;
    
  while (ros::ok()){
    
    ros::spinOnce();
    init_tau();

    if(count > 25)
      control_tau.tau.force.x = 10;
      
    
    /*if(count < 50)
      control_tau.tau.force.x = -10;*/

      

      
    control_pub.publish(control_tau);


    loop_rate.sleep();
    count++;
  }

return 0;
}
