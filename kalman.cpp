/*
Input: Topic /sensor
Elaborazione:
>Predizione con meccanizzazione
>Correzione con /sensor
>Conversione in lat. long.
Output: /odom

my_/odom:
>Lat. Long. Quota.
>RPY
>Velocità lineare
>Velocità angolare
*/

/*
/sensor: 
Acc. 
Gyro.

Magn.
Depth
USBL
DVL
*/


#include "ros/ros.h"
#include <sstream>
//#include "nav_msgs/Odometry.h"
#include <math.h>
//#include "sensor_msgs.msg"
//#include "my_odometry_msgs.msg"

class Vector
{
public:
	//variabili
	float x;
	float y;
	float z;
	
	Vector(float x_new, float y_new, float z_new); //costruttore
	
	void set_v(float x_new, float y_new, float z_new); //metodi
	void set_Vect(Vector v_new);
	void scalare(float  k);
	void sum_v(Vector v2);
};

Vector::Vector(float x_new, float y_new, float z_new)	//to do: default constructor cpp
{
	x=x_new;
	y=y_new;
	z=z_new;
}

void Vector::set_v(float x_new, float y_new, float z_new)
{
	x=x_new;
	y=y_new;
	z=z_new;

}

void Vector::set_Vect(Vector v_new)
{
	x=v_new.x;
	y=v_new.y;
	z=v_new.z;
}

void Vector::scalare(float k)
{
	x=k*x;
	y=k*y;
	z=k*z;
}

void sum_v(Vector v2)
{
	x = x + v2.x;
	y = y + v2.y;
	z = z + v2.z;	
}

//Variabili
Vector eta1(0, 0, 0);		//pos. in NED
Vector eta2(0, 0, 0);		//orientamento RPY
Vector eta1_dot(0, 0, 0);	//velocità lin. in NED
Vector eta2_dot(0, 0, 0);	//vel. angolare RPY

/**** Sensori ****/
//Inerziale
Vector acc(0, 0, 0);
Vector gyro(0, 0, 0);
//Non inerziale
Vector usbl(0, 0, 0);	//posizione
Vector dvl(0, 0, 0);	//velocità lin.
Vector magn(0, 0, 0);	//orientamento
float depth=0;		//profondità
#define f 100
const int T=1/f;

void predict(Vector, Vector, Vector, Vector);
void correction(Vector, Vector, Vector, Vector, float); 

void sensor_callback(const sensor_msg::ConstPtr& msg)
{
	//update
	acc.set_v(msg->acc.x, msg->acc.y, msg->acc.z);
	gyro.set_v(msg->gyro.x, msg->gyro.y, msg->gyro.z);
	usbl.set_v(msg->usbl.x, msg->usbl.y, msg->usbl.z);
	dvl.set_v(msg->dvl.x, msg->dvl.y, msg->dvl.z);
	magn.set_v(msg->magn.x, msg->magn.y, msg->magn.z);
	depth=msg->depth.data;

	//Debug
	ROS_INFO("Dati ricevuti\n");

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Kalman");
	ros::NodeHandle node_obj;
	
	//Pub
	ros::Publisher pub_obj=node_obj.advertise<my_odometry_msgs>("/myodom", 1000);
	//Sub
	ros::Subscriber subscriber_obj=node_obj.subscribe("/sensor", 1000, sensor_callback);
	ros::Rate loop_rate(T);	//frequenza di predizione 

	while(ros::ok())
	{
		ros::spinOnce(f);

		predict(); //Input: acc, gyro e stati precedenti
		correction(); 
		
			
		
	}


}

void predict()
{
	/* TO DO: Fare variabili locali per i calcoli della predizione
		  e poi aggiorni negli oggetti globali eta1, eta2 ecc */
	
		
};

void correction()
{
	//idem
};




