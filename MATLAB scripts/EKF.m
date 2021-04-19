clear all
clc

%% Sez.1
syms x y z phi theta psi u v w p q r 'real'

eta1=[x y z]';
eta2=[phi theta psi]';
eta=[eta1; eta2];
ni1=[u v w]';
ni2=[p q r]';
ni=[ni1; ni2];

c_phi=cos(phi);
s_phi=sin(phi);
c_theta=cos(theta);
s_theta=sin(theta);
t_theta=tan(theta);
c_psi=cos(psi);
s_psi=sin(psi);

Rx=[1 0 0; 0 c_phi s_phi; 0 -s_phi c_phi]';
Ry=[c_theta 0 s_theta; 0 1 0; -s_theta 0 c_theta]';
Rz=[c_psi s_psi 0; -s_psi c_psi 0; 0 0 1]';

J1=simplify(Rz*Ry*Rx);

J2=[1 s_phi*t_theta c_phi*t_theta; 
    0 c_phi -s_phi; 
    0 s_phi/c_theta, c_phi/c_theta];

syms phi_dot theta_dot psi_dot 'real'
eta2_dot=[phi_dot theta_dot psi_dot]';
%%%test
% r11= -psi_dot*s_psi*c_theta -theta_dot*c_psi*s_theta;
% r12= phi_dot*s_phi*s_psi -psi_dot*c_phi*c_psi + psi_dot*s_psi*s_phi*s_theta -phi_dot*c_psi*c_phi*s_theta - theta_dot*c_psi*s_phi*c_theta;
% r13= phi_dot*c_phi*s_psi + psi_dot*s_phi*c_psi + phi_dot*s_phi*c_psi*s_theta + psi_dot*c_phi*s_psi*s_theta - theta_dot*c_phi*c_psi*c_theta;
% r21=-theta_dot*s_theta*s_psi + psi_dot*c_theta*c_psi;
% r22=-phi_dot*s_phi*c_psi-psi_dot*c_phi*s_psi -phi_dot*c_phi*s_psi*s_theta-psi_dot*s_phi*c_psi*s_theta-theta_dot*s_phi*s_psi*c_theta;
% r23=psi_dot*s_psi*s_phi - phi_dot*c_phi*c_psi +phi_dot*s_phi*s_psi*s_theta-psi_dot*c_phi*c_psi*s_theta-theta_dot*c_phi*s_psi*c_theta;
% r31=theta_dot*c_theta;
% r32=-theta_dot*s_theta*s_phi + phi_dot*c_theta*c_phi;
% r33=-phi_dot*s_phi*c_theta - theta_dot*c_phi*s_theta;
% R=[r11 r12 r13; r21 r22 r23; r31 r32 r33];
% Test=simplify(R*(J1'));
% w=[Test(3, 2); Test(2, 1); Test(1, 3)];
transf=[c_psi*c_theta s_psi 0; s_theta 0 1; c_theta*s_psi -c_psi 0];
w=transf*eta2_dot;
%% Eq di Navigazione
syms T acc_x acc_y acc_z gyro_x gyro_y gyro_z x_dot y_dot z_dot 'real'
syms w_acc_x w_acc_y w_acc_z w_gyro_x w_gyro_y w_gyro_z 'real'
eta1_dot=[x_dot; y_dot; z_dot];
x1=eta1;
x2=eta2;
x3=eta1_dot;
state=[x1; x2; x3];
u1=[acc_x; acc_y; acc_z];
u2=[gyro_x; gyro_y; gyro_z];
w_acc=[w_acc_x; w_acc_y; w_acc_z];
w_gyro=[w_gyro_x; w_gyro_y; w_gyro_z];
w_imu= [w_acc; w_gyro];

%Eq alle differenze
x1_upd=x1 + T*x3;
x2_upd=x2 + T*J2*u2 + T*w_gyro;
x3_upd=(eye(3) + T*skew_product(transf*(J2*u2 +w_gyro)))*x3+ T*J1*w_acc +T*J1*u1;

state_upd=[x1_upd; x2_upd; x3_upd];

F=simplify(subs(diff_matrix_by_vector(state_upd, state), w_imu, zeros(6, 1)));
D=simplify(subs(diff_matrix_by_vector(state_upd, w_imu), w_imu, zeros(6, 1)));

syms P [9 9] matrix 
syms Q [6 6] matrix 
P=symmatrix2sym(P);
Q=symmatrix2sym(Q);

x_pred=subs(state_upd, w_imu, zeros(6,1));
P_pred=F*P*(F') + D*Q*(D');


function S=skew_product(x)
S=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end