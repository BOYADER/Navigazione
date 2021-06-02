#include "ros/ros.h"
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "modellazione/depth.h"
#include "modellazione/state_real.h"
#include "math_utility.h"
#include "sensor_utility.h"
#include <cmath>
#include <random>

using namespace Eigen;
using namespace std;

modellazione::depth depth_measure;

void depth_state_read(const modellazione::state_real &state)
{
  //NON E' STATA SUPPOSTA ALCUNA SEMPLIFICAZIONE RIGUARDO POSIZIONE DEL PROFONDIMETRO
  Matrix3f J_inv = compute_jacobian1(state.eta_2).transpose();
  float p_depth_z = (J_inv * p_depth)(2);
  depth_measure.z = state.eta_1.z; // + p_depth_z; 
  depth_measure.counter++;         
}



int main(int argc, char **argv){

  ros::init(argc, argv, "depth_sensor");

  ros::NodeHandle depth_sensor;

  ros::Subscriber depth_sub = depth_sensor.subscribe("state_real", 1, depth_state_read);
  ros::Publisher depth_pub = depth_sensor.advertise<modellazione::depth>("sensor/depth", MAX_QUEUE_LENGTH);

  default_random_engine generator;
  normal_distribution<double> depth_distribution(0, 2e-3);

  ros::Rate loop_rate(SENSOR_FREQUENCY);
  ros::spinOnce();
  loop_rate.sleep();

  while(ros::ok()){
    float old_counter = depth_measure.counter;
  	ros::spinOnce();

    if(old_counter != depth_measure.counter){
      depth_measure.z += depth_distribution(generator);
      //ROS_INFO("sto per pubblicare: \n depth = [%f] \n counter = [%f]", depth_measure.z, depth_measure.counter);
      depth_pub.publish(depth_measure);
    }
  
    loop_rate.sleep();

  }

  return 0;
}