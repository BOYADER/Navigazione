#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <sstream>
#include "/usr/include/eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "modellazione/state_real.h"
#include "modellazione/tau.h"
#include "constant.h"
#include "math_utility.h"

/*  Model node must subscribe /tau topic in order to obtain control actions.
    After solving dynamics differential equations it has to publish on 
    /state_real topic, from which every sensor will subscribe and take the 
    actual informations needed.
*/      
 
using namespace Eigen;

geometry_msgs::Vector3 dyn_force;
geometry_msgs::Vector3 dyn_torque;
modellazione::state_real state;
geometry_msgs::Vector3 eta_1_dot_dot;

//calcolo elementi matrici masse aggiunte

float e = 1 - (R_B/R_A) * (R_B/R_A);
float alpha_0 = 2 * (1 - e * e) * ( 0.5 * log((1 + e) / (1 - e)) - e) / (e * e * e);
float beta_0 = 1/(e*e) - (1 - e*e)* log((1+e)/(1-e)) / (2*e*e*e);

//float X_u_dot = - alpha_0 / (2 - alpha_0) * R_M;
float Y_v_dot = - beta_0 / (2 - beta_0) * R_M;
float X_u_dot = Y_v_dot;
float Z_w_dot = Y_v_dot;

float K_p_dot = 0;
float N_r_dot = (-1.0/5*R_M)* ( R_B*R_B - R_A*R_A)*( R_B*R_B - R_A*R_A)*(alpha_0 - beta_0)/(2.0*(R_B*R_B - R_A*R_A) + (R_B*R_B + R_A*R_A)*(beta_0 - alpha_0));
float M_q_dot = (-1.0/5*R_M)* ( R_B*R_B - R_A*R_A)*( R_B*R_B - R_A*R_A)*(alpha_0 - beta_0)/(2.0*(R_B*R_B - R_A*R_A) + (R_B*R_B + R_A*R_A)*(beta_0 - alpha_0));



float compute_damping(u_int lato, float ni_i) // che area prendiamo nei vari casi? forse quella laterale sempre?
{
  float area, b, b1, b2, contr1, contr2;
  switch(lato){
    case 1:
      area = 2 * R_A * 2 * R_B;
      return (-1.0/2) * RHO_W * area *C_D_X* abs(ni_i);
    
    case 2:
      area = 2 * R_B * 2 * R_C; 
      return (-1.0/2) * RHO_W * area *C_D_YZ * abs(ni_i);
    
    case 3:
      area = R_B * 2 * R_C; 
      b = sqrt(R_A*R_A/4.0 + R_B*R_B/4.0);
      return (-1.0/2) * RHO_W * area *C_D_YZ * abs(ni_i)*b*b*b * 2;  // si moltiplica per due per tener conto dei due contributi relativi ai due baricentri "intermedi"
    
    case 4:
      area = 2 * R_A * 2 * R_B;
      b1 = (R_B + R_B/2.0) /2.0;
      b2 = R_B / 4.0;
      contr1 = (-1.0/2) * RHO_W * area*3/4.0 *C_D_X * abs(ni_i)*b1*b1*b1;
      contr2 = (-1.0/2) * RHO_W * area/4.0 *C_D_X * abs(ni_i)*b2*b2*b2;
      return contr1 + contr2;

    default:
      break;
  }
}

// matrici della dinamica 

MatrixXf M(6,6);
MatrixXf D(6,6);
MatrixXf C(6,6);
VectorXf G(6);



void initializeM(){
  M(0,0) = R_M - X_u_dot;
  M(1,1) = R_M - Y_v_dot;
  M(2,2) = R_M - Z_w_dot;
  M(3,3) = I_X;
  M(4,4) = I_Y;
  M(5,5) = I_Z;
}

void updateD(){
  D(0,0) = - 10*compute_damping(1, state.ni_1.x);
  D(1,1) = - 10*compute_damping(2, state.ni_1.y);
  D(2,2) = - 10*compute_damping(2, state.ni_1.z);
  D(3,3) = - 100*compute_damping(4, state.ni_2.x);
  D(4,4) = - 100*compute_damping(3, state.ni_2.y);
  D(5,5) = - 100*compute_damping(3, state.ni_2.z);
}

void updateC(){
  C(0,4) = - Z_w_dot * state.ni_1.z;
  C(0,5) =   Y_v_dot * state.ni_1.y;
  C(1,3) =   Z_w_dot * state.ni_1.z;
  C(1,5) = - X_u_dot * state.ni_1.x;
  C(2,3) = - Y_v_dot * state.ni_1.y;
  C(2,4) =   X_u_dot * state.ni_1.x;
  C(3,1) = - Z_w_dot * state.ni_1.z;
  C(3,2) =   Y_v_dot * state.ni_1.y;
  C(4,0) =   Z_w_dot * state.ni_1.z;
  C(4,2) = - X_u_dot * state.ni_1.x;
  C(5,0) = - Y_v_dot * state.ni_1.y;
  C(5,1) =   X_u_dot * state.ni_1.x;
  C(3,4) = - N_r_dot * state.ni_2.z;
  C(3,5) =   M_q_dot * state.ni_2.y;
  C(4,3) =   N_r_dot * state.ni_2.z;
  C(4,5) = - K_p_dot * state.ni_2.x;
  C(5,3) = - M_q_dot * state.ni_2.y;
  C(5,4) =   K_p_dot * state.ni_2.x;
 
}


void updateG(){
  Matrix3f J_inv = compute_jacobian1(state.eta_2).transpose();
  Vector3f f_g = J_inv * weight;
  Vector3f f_b = J_inv * buoyancy;
  G.head(3) = -(f_g + f_b);
  G.tail(3) = - (r_b.cross(f_b));
}


void overwater_check(Vector3f ni1){
	if (state.eta_1.z < 0){
    	// Aggiustiamo la posizione
  		state.eta_1.z = 0; 
    	// Per annullare la componente verticale
    	// della velocità dobbiamo passare in terna NED 
    	// e poi di nuovo in terna body
    	Vector3f eta1_dot = compute_jacobian1(state.eta_2) * ni1;
    	eta1_dot(2) = 0;
    	Vector3f ni1_checked = compute_jacobian1(state.eta_2).transpose()*eta1_dot;
    	state.ni_1 = eigen2ros(ni1_checked);
    	// Aggiustiamo l'accelerazione
    	state.eta_1_dot_dot.z = 0;
    	return;
  	}
  	else
  		return;
}


void resolve_dynamics(){
  static ros::Time old_time;
  static int count;

  ros::Time new_time = ros::Time::now();
  double dt;
  if(count == 0)
    dt = (1.0/MODEL_FREQUENCY); 
  else
    dt = new_time.toSec() - old_time.toSec();
  std:: cout <<"dt = " << dt << std::endl <<std::endl;

  updateD();
  updateG();
  updateC();

  Vector3f ni1 = ros2eigen(state.ni_1);
  Vector3f ni2 = ros2eigen(state.ni_2);
  Vector3f eta1 = ros2eigen(state.eta_1);
  Vector3f eta2 = ros2eigen(state.eta_2);
  VectorXf ni(6);
  VectorXf eta(6);
  VectorXf tau(6);
  ni.head(3) = ni1;
  ni.tail(3) = ni2;
  eta.head(3) = eta1;
  eta.tail(3) = eta2;
  tau.head(3) = ros2eigen(dyn_force);
  tau.tail(3) = ros2eigen(dyn_torque);

  // compute ni
  VectorXf new_ni(6);
  new_ni = ni + dt * M.inverse() * (tau - C*ni - D*ni - G);
  std::cout << "tau = \n" << tau << std::endl << std::endl;
  std::cout << "C*ni = \n" << C*ni << std::endl << std::endl;
  std::cout << "D*ni = \n" << D*ni << std::endl << std::endl;
  std::cout << "G = \n" << G << std::endl << std::endl;

  //  compute eta
  VectorXf new_eta(6);
  new_eta = eta + dt * compute_jacobian_tot(state.eta_2) * ni;
  //  compute eta_dot_dot
  Vector3f ni1_dot = M.inverse().block<3,3>(0,0) * (tau.head(3) - C.block<3,3>(0,0)*ni1 - D.block<3,3>(0,0)*ni1 - G.head(3));
  Vector3f eta1_dot_dot = compute_jacobian1(state.eta_2) * ni1_dot + compute_jacobian1(state.eta_2)*skew_symmetric(state.ni_2)*ni1;
  // update state
  state.eta_1_dot_dot = eigen2ros(eta1_dot_dot);
  state.eta_1 = eigen2ros(new_eta.head(3));
  state.eta_2 = eigen2ros(new_eta.tail(3));
  state.ni_1 = eigen2ros(new_ni.head(3));
  state.ni_2 = eigen2ros(new_ni.tail(3));

  overwater_check(ni1);

  old_time = new_time;
  count++;

}


void tau_read(const modellazione::tau &wrench){
  dyn_force = wrench.tau.force;
  dyn_torque = wrench.tau.torque;
  /*ROS_INFO("I heard force = [%f %f %f] \n torque = [%f %f %f] \n", 
            dyn_force.x, dyn_force.y, dyn_force.z,
            dyn_torque.x, dyn_torque.y, dyn_torque.z);*/
}


float constrain_angle(double x){
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

//NOTA: perdiamo traccia di eventuali giri compiuti dal robot
void normalize_angles(){
	state.eta_2.x = constrain_angle(state.eta_2.x);
	state.eta_2.y = constrain_angle(state.eta_2.y);
	state.eta_2.z = constrain_angle(state.eta_2.z);
	return;
}



int main(int argc, char **argv)   
{
  ros::init(argc, argv, "model");
  ros::NodeHandle model;

  ros::Subscriber model_sub = model.subscribe("tau", 1, tau_read);
  ros::Publisher model_pub = model.advertise<modellazione::state_real>("state_real", MAX_QUEUE_LENGTH);

  /* Raccolta info su stato iniziale come definito nel file mission.yaml */
  float yaw0, pitch0, roll0;
  model.getParam("/initial_pose/orientation/yaw",     yaw0);
  model.getParam("/initial_pose/orientation/pitch",   pitch0);
  model.getParam("/initial_pose/orientation/roll",    roll0);
  state.eta_2.x = roll0;  
  state.eta_2.y = pitch0;
  state.eta_2.z = yaw0;
  /*----------------------------------------------------------------------*/

  ros::Rate loop_rate(MODEL_FREQUENCY);
  
  float count = 1;

  initializeM();
  std::cout << "eccentricità = " << e << std::endl;
  std::cout << "alpha0 = " << alpha_0 << std::endl;
  std::cout << "beta0 = " << beta_0 << std::endl;
  std::cout << "Matrice di massa:\n" << M << std::endl;
    
  while (ros::ok()){
    
    ros::spinOnce();
    
    resolve_dynamics();
    //std::cout << "MATRICE DI DAMPING: \n" << D << std::endl << std::endl;
    //std::cout << "MATRICE DI CORIOLIS: \n" << C << std::endl << std::endl;
    //std::cout << "VETTORE G: \n" << G << std::endl << std::endl;
   	normalize_angles();

    state.prova = count++;
    
    ROS_INFO("Sto per pubblicare \neta1 = [%f %f %f] \nni1 = [%f %f %f] \neta1_dot_dot= [%f %f %f] \neta2 = [%f %f %f] \nni2 = [%f %f %f] \nmsg numero: %f \n",
              state.eta_1.x, state.eta_1.y, state.eta_1.z,
              state.ni_1.x, state.ni_1.y, state.ni_1.z,
              state.eta_1_dot_dot.x,state.eta_1_dot_dot.y,state.eta_1_dot_dot.z,
              state.eta_2.x, state.eta_2.y, state.eta_2.z,
              state.ni_2.x, state.ni_2.y, state.ni_2.z,
              state.prova);

    model_pub.publish(state);

    loop_rate.sleep();
  }

return 0;
}


