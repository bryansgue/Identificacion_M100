function [xp] = f_dynamics(h, u, chi)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
q = h(1:6);
q_p = h(7:12);

g = 9.81;
m = chi(1);

Mbar = M_matrix_bar(chi,q);
Cbar = C_matrix_bar(chi,q,q_p);
Gbar = G_matrix_bar(chi,q);

 B = [0;0;m*g;0;0;0];

R = Rot_zyx(q(4:6));
% 
S = S_fuction(chi);
Q = Q_fuction(chi);
E = E_fuction(chi);
T = T_fuction(chi);

R_bar = [R zeros(3,3);
        zeros(3,3) eye(3,3)];


Aux = (S*u-Q*q-E*q_p+B);

q_pp = pinv(Mbar+R_bar*T)*(R_bar*Aux-Cbar*q_p-Gbar);

xp = [q_p; q_pp];

end

