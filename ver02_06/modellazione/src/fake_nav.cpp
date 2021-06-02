#include <iostream>
#include "/usr/include/eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include "modellazione/state_real.h"
#include "modellazione/dvl.h"
#include "modellazione/gps.h"
#include "modellazione/usbl.h"
#include "modellazione/ahrs.h"
#include "modellazione/depth.h"
#include <sstream>
#include "constant.h"
#include <math.h>

//QUESTO NODO DEVE SIMULARE IL SUBSCRIBE SUI TOPIC
// DEI VARI SENSORI
void ahrs_callback(const modellazione::ahrs &ahrs_measure){


}

void usbl_callback(const modellazione::usbl &usbl_measure){


}

void dvl_callback(const modellazione::dvl &dvl_measure){


}

void depth_callback(const modellazione::depth &depth_measure){


}

void gps_callback(const modellazione::gps &gps_measure){


}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigazione");
	ros::NodeHandle fake_nav;
	
	ros::Subscriber sub_ahrs=fake_nav.subscribe("sensor/ahrs", 1000, ahrs_callback);
	ros::Subscriber sub_usbl=fake_nav.subscribe("sensor/usbl", 1000, usbl_callback);
	ros::Subscriber sub_dvl=fake_nav.subscribe("sensor/dvl", 1000, dvl_callback);
	ros::Subscriber sub_depth=fake_nav.subscribe("sensor/depth", 1000, depth_callback);
	ros::Subscriber sub_gps=fake_nav.subscribe("sensor/gps", 1000, gps_callback);
	ros::Rate loop_rate(1);	

	while (ros::ok()){
    
    	ros::spinOnce();

    	loop_rate.sleep();
  }

return 0;
}







