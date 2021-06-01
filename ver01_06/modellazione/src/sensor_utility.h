#include "/usr/include/eigen3/Eigen/Dense"
#include <cmath>

using namespace Eigen;



//PROFONDIMETRO
#define R_PROFONDIMETRO 0.08 //distanza tra centro di massa e profondimetro [m]
Vector3f p_depth(0, 0, 0);



//DVL
#define R_DVL_X 0.37 //coordinata x del dvl rispetto alla terna body
#define R_DVL_Z 0.03 //coordinata z del dvl rispetto alla terna body
#define THETA_DVL 0 //rotazione relativa tra terna body e dvl 
Vector3f p_dvl(0, 0, 0); 
MatrixXf R_dvl_body(3,3);

void initialise_R_dvl_body(){
	R_dvl_body(0,0) = cos(THETA_DVL); 
	R_dvl_body(0,1) = 0;
	R_dvl_body(0,2) = -sin(THETA_DVL);
	R_dvl_body(1,0) = 0;
	R_dvl_body(1,1) = 1;
	R_dvl_body(1,2) = 0;
	R_dvl_body(2,0) = sin(THETA_DVL); 
	R_dvl_body(2,1) = 0;
	R_dvl_body(2,2) = cos(THETA_DVL);
}


//USBL: supponiamo inizialmente che terna usbl coincida con terna body
//#define R_USBL_Z 0.08 //[m]; l'usbl si trova sotto il centro di massa, sulla base del veicolo

#define R_USBL_Z 0.0   //usbl nel centro di massa
#define R_T 50        //[m]; profondita' del transponder 
Vector3f p_usbl(0, 0, R_USBL_Z);
Vector3f p_t_ned(0, 0, R_T); //vettore che rappresenta la posizione del transponder
							 //espresso in terna NED