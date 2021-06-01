#include "ros/ros.h"
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "modellazione/state_real.h"
#include "modellazione/dvl.h"
#include "math_utility.h"
#include "sensor_utility.h"
#include <random>


using namespace Eigen;
using namespace std;

modellazione::dvl dvl_measure;
double std_dev_x, std_dev_y, std_dev_z;

void dvl_state_read(const modellazione::state_real &state)
{
  Vector3f ni1 = ros2eigen(state.ni_1);
  Matrix3f S = skew_symmetric(state.ni_2);
  dvl_measure.lin_vel = eigen2ros( R_dvl_body *(ni1 + S*p_dvl) ); 
  dvl_measure.counter++; 
  std_dev_x = 0.01 * ni1(0) + 0.01;
  std_dev_y = 0.01 * ni1(1) + 0.01;
  std_dev_z = 0.01 * ni1(2) + 0.01;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "dvl_sensor");

  ros::NodeHandle dvl_sensor;

  ros::Subscriber dvl_sub = dvl_sensor.subscribe("state_real", 1, dvl_state_read);
  ros::Publisher dvl_pub = dvl_sensor.advertise<modellazione::dvl>("sensor/dvl", MAX_QUEUE_LENGTH);
  initialise_R_dvl_body();

  default_random_engine generator;
  
  ros::Rate loop_rate(1);

  while(ros::ok()){ 

    ros::spinOnce();

    normal_distribution<double> dvl_distribution_x(0, std_dev_x);
    normal_distribution<double> dvl_distribution_y(0, std_dev_y);
    normal_distribution<double> dvl_distribution_z(0, std_dev_z);

    dvl_measure.lin_vel.x += dvl_distribution_x(generator);
    dvl_measure.lin_vel.y += dvl_distribution_y(generator);
    dvl_measure.lin_vel.z += dvl_distribution_z(generator);

    ROS_INFO("Sto per pubblicare misura dvl: \n x = [%f] \n y = [%f] \n z = [%f] ", 
              dvl_measure.lin_vel.x, dvl_measure.lin_vel.y, dvl_measure.lin_vel.z);

    dvl_pub.publish(dvl_measure);

    loop_rate.sleep();

  }


  return 0;
}
