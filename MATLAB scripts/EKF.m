clear all
clc

%% Variabili simboliche
syms x y z phi theta psi u v w p q r 'real'
syms T acc_x acc_y acc_z gyro_x gyro_y gyro_z x_dot y_dot z_dot 'real'
syms w_acc_x w_acc_y w_acc_z w_gyro_x w_gyro_y w_gyro_z 'real'
syms w_ahrs_x w_ahrs_y w_ahrs_z 'real'

eta1=[x y z]';
eta2=[phi theta psi]';
eta1_dot=[x_dot; y_dot; z_dot];

%syms u v w p q r 'real'
syms w_ni1_x w_ni1_y w_ni1_z 'real'
syms w_eta2_x w_eta2_y w_eta2_z 'real'
syms w_eta1_x w_eta1_y w_eta1_z 'real'

ni1 = [u; v; w];

w_eta1 = [w_eta1_x; w_eta1_y; w_eta1_z];
w_ahrs=[w_ahrs_x; w_ahrs_y; w_ahrs_z];
w_ni1= [w_ni1_x; w_ni1_y; w_ni1_z];
w_eta2 = [w_eta2_x; w_eta2_y; w_eta2_z];
w= [w_eta1; w_ni1; w_eta2];

x1= eta1;
x2= ni1;
x3= eta2;

state=[x1; x2; x3];
J1=jacobian1(x3);

x1_pred = x1 + T*J1*x2 +T*w_eta1;
x2_pred= x2 + T*w_ni1;
x3_pred = x3 + T*w_eta2;

state_pred=simplify([x1_pred; x2_pred; x3_pred]);

F=simplify(subs(diff_matrix_by_vector(state_pred, state), w, zeros(length(w), 1)));
D=simplify(subs(diff_matrix_by_vector(state_pred, w), w, zeros(length(w), 1)));

%% Eq. Sensori non inerziali
syms v_usbl_x v_usbl_y v_usbl_z 'real'
syms v_dvl_x v_dvl_y v_dvl_z 'real'
syms v_depth DEPTH 'real'
syms v_gyro_x v_gyro_y v_gyro_z 'real'
syms px py pz 'real'

v_usbl = [v_usbl_x; v_usbl_y; v_usbl_z];
v_dvl = [v_dvl_x; v_dvl_y; v_dvl_z];
v_gyro = [v_gyro_x; v_gyro_y; v_gyro_z];
v= [v_usbl; v_dvl; v_depth; w_ahrs; v_gyro];

R_dvl = eye(3);
p_dvl = [px; py; pz]; 

%usbl
p_usbl=[0 0 DEPTH]';

eta_b = simplify((J1')*(p_usbl - x1));
r= sqrt(eta_b(1)^2 + eta_b(2)^2 + eta_b(3)^2);
b= atan2(eta_b(2), eta_b(1));
e= acos(eta_b(3)/r);
y_usbl=simplify([r; b; e]) +v_usbl;

%dvl
y_dvl = R_dvl*(x2 - skew_product(v_gyro)*p_dvl) + v_dvl;

%depth
y_depth = x1(3) + v_depth;

%ahrs
y_ahrs = x3 +w_ahrs;

output = simplify([y_usbl; y_dvl; y_depth; y_ahrs]);

H=simplify(subs(diff_matrix_by_vector(output, state), v, zeros(length(v), 1)));
M=simplify(subs(diff_matrix_by_vector(output, v), v, zeros(length(v), 1)));


