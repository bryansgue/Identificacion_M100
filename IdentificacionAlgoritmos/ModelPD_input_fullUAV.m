function [T] = ModelPD_input_fullUAV(chi,u_ref, euler, euler_p, euler_pp)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

phi_ref = u_ref(1);
theta_ref = u_ref(2);
psi_p_ref = u_ref(3);

phi = euler(1);
theta = euler(2);
psi = euler(3);

phi_p = euler_p(1);
theta_p = euler_p(2);
psi_p = euler_p(3);

phi_pp = euler_pp(1);
theta_pp = euler_pp(2);
psi_pp = euler_pp(3);

c_1 = phi_p*theta_p;
c_2 = phi_p*psi_p;
c_3 = theta_p*psi_p;

c = [c_1;c_2;c_3];

% chix =    1.0e-05 * [ -0.1552   -0.0305    0.0444    0.0145   -0.0444] ;
% chiy =    1.0e-04 * [ -0.1389   -0.0240    0.0588   -0.0027    0.0076] ;
% chiz =    [0.0057    0.0001    0.0001];
% 
% chif = [chix,chiy,chiz];
% 
% tau_x = chi(1)*(phi_ref-phi) - chi(2)*phi_p  - chi(3)*c_1 - chi(4)*c_2 - chi(5)*c_3;
% tau_y = chi(6)*(theta_ref- theta) - chi(7)*theta_p  - chi(8)*c_1 - chi(9)*c_2 - chi(10)*c_3 ;
% tau_z = chi(11)*(psi_p_ref-psi_p);


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
% %  
T = A*u_ref - B*euler - C*euler_p - D*c;

% T = [tau_x;tau_y;tau_z];

end