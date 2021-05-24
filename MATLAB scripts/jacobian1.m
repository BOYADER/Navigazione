function J1=jacobian1(rpy_with_error)
phi=rpy_with_error(1);
theta=rpy_with_error(2);
psi=rpy_with_error(3);

c_phi=cos(phi);
s_phi=sin(phi);
c_theta=cos(theta);
s_theta=sin(theta);
c_psi=cos(psi);
s_psi=sin(psi);

Rx=[1 0 0; 0 c_phi s_phi; 0 -s_phi c_phi]';
Ry=[c_theta 0 -s_theta; 0 1 0; s_theta 0 c_theta]';
Rz=[c_psi s_psi 0; -s_psi c_psi 0; 0 0 1]';

J1=simplify(Rz*Ry*Rx);
end