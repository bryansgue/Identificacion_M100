function [x_p] = f_dynamics_UAV_Model(x, u, chi)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
euler = x(1:3);
euler_p = x(4:6);



M = M_matrix(chi,euler);
C = C_matrix(chi,euler,euler_p);

R = Rot_zyx(euler);
% 
S = zeros(3,3);
S(1,1) = chi(4);
S(2,2) = chi(5);
S(3,3) = chi(6);

Q = zeros(3,3);
Q(1,1) = chi(7);
Q(2,2) = chi(8);
Q(3,3) = 0;

E = zeros(3,3);
E(1,1) = chi(9);
E(2,2) = chi(10);
E(3,3) = chi(11);

L = zeros(3,3);
L(1,1) = chi(12);
L(2,2) = chi(13);
L(3,3) = chi(14);

q_p = euler_p;
q_pp = inv(M-R*L)*R*(S*u+Q*euler+(E-C)*euler_p);



x_p = [q_p; q_pp];

end

