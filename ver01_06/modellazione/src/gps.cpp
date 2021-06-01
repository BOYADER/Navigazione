#include "ros/ros.h"
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "modellazione/gps.h"
#include "modellazione/state_real.h"
#include "math_utility.h"
#include <random>

using namespace std;
using namespace Eigen;

modellazione::gps gps_measure;
geometry_msgs::Vector3 eta1;


void gps_state_read(const modellazione::state_real &state)
{
  eta1 = state.eta_1;
  if(eta1.z >= 0.1)
    gps_measure.under_water = true;
  else 
    gps_measure.under_water = false;
  gps_measure.counter++;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "gps_sensor");

  ros::NodeHandle gps_sensor;

  ros::Subscriber gps_sub = gps_sensor.subscribe("state_real", 1, gps_state_read);
  ros::Publisher gps_pub = gps_sensor.advertise<modellazione::gps>("sensor/gps", MAX_QUEUE_LENGTH);

  //Generazione Rumore
  default_random_engine generator;
  normal_distribution<double> gps_distribution(0, 1); //[m]
  Vector2f LL;

  //DATI NECESSARI PER LA CONVERSIONE NED => LL /////////////////
  float lat0, lon0, a0;
  gps_sensor.getParam("/initial_pose/position/latitude",   lat0);
  gps_sensor.getParam("/initial_pose/position/longitude",  lon0);
  gps_sensor.getParam("/initial_pose/position/depth",      a0);
  geometry_msgs::Vector3 pos_0;
  pos_0.x = lat0;
  pos_0.y = lon0;
  pos_0.z = a0;
  ///////////////////////////////////////////////////////////////

  ros::Rate loop_rate(1);

  while(ros::ok()){

  	ros::spinOnce();
    
   	eta1.x += gps_distribution(generator);
   	eta1.y += gps_distribution(generator);
    
    LL = NEDtoLL_conversion(eta1, pos_0);
    gps_measure.lla.x = LL(0);
    gps_measure.lla.y = LL(1);

    gps_pub.publish(gps_measure);

    loop_rate.sleep();

  }

  return 0;
}