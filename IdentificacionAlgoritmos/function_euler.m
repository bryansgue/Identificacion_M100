function [x_p] = function_euler(x, u_ref, chi)
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

c = [c_1;c_2;c_3];


A = [chi(1) 0 0;
     0 chi(6) 0;
     0 0 chi(11)];
B = [chi(1) 0 0;
     0 chi(6) 0;
     0 0 0];
C = [chi(2) 0 0;
     0 chi(7) 0;
     0 0 chi(11)];
D = [chi(3) chi(4) chi(5);
     chi(8) chi(9) chi(10);
     0 0 0];
 

chiI =    1.0e-06 * [ -0.0406 -0.2634 0.2445];
 
Mbar = M_matrix(chiI,euler);
Cbar = C_matrix(chiI,euler,euler_p);

x1_p = x2;

x2_p = pinv(Mbar)*(A*u_ref - B*euler - C*euler_p - D*c - Cbar*euler_p );


x_p = [x1_p; x2_p];

end

