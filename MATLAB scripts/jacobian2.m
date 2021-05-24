function J2=jacobian2(rpy)
phi=rpy(1);
theta=rpy(2);
psi=rpy(3);

c_phi=cos(phi);
s_phi=sin(phi);
c_theta=cos(theta);
s_theta=sin(theta);
t_theta=tan(theta);
c_psi=cos(psi);
s_psi=sin(psi);

J2=[1 s_phi*t_theta c_phi*t_theta; 0 c_phi -s_phi; 0 s_phi/c_theta c_phi/c_theta]; 
end