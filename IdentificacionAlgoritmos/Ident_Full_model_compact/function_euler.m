function [x_p] = function_euler(x, T_ref, chi)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
x1 = x(1:3);
x2 = x(4:6);

euler = x1;
euler_p = x2;

phi_p = euler_p(1);
theta_p = euler_p(2);
psi_p = euler_p(3);

c_1 = phi_p*theta_p;
c_2 = phi_p*psi_p;
c_3 = theta_p*psi_p;

sigma = [c_1;c_2;c_3];



Mbar = M_matrix(chi,euler);
Cbar = C_matrix(chi,euler,euler_p);

A = [chi(4) 0 0;
     0 chi(11) 0;
     0 0 chi(18)];
B = [chi(5) 0 0;
     0 chi(12) 0;
     0 0 chi(24)];
C = [chi(6) 0 0;
     0 chi(13) 0;
     0 0 chi(19)];
D = [chi(7) chi(8) chi(9);
     chi(14) chi(15) chi(16);
     chi(20) chi(21) chi(22)];
E = [chi(10) 0 0;
     0 chi(17) 0;
     0 0 chi(23)];

x1_p = x2;

x2_p = pinv(Mbar+E)*(A*T_ref + B*euler + C*euler_p - Cbar*euler_p + D*sigma);

x_p = [x1_p; x2_p];

end

