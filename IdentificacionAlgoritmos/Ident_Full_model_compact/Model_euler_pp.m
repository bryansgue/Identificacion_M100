function [euler_pp,A,B,C,D] = Model_euler_pp(chi,T_ref,euler,euler_p)

phi_p = euler_p(1);
theta_p = euler_p(2);
psi_p = euler_p(3);

c_1 = phi_p*theta_p;
c_2 = phi_p*psi_p;
c_3 = theta_p*psi_p;

sigma = [c_1;c_2;c_3];


% chiI = [0.9289    2.5456    1.5625];
% 
% Mbar = M_matrix(chiI,euler);
% Cbar = C_matrix(chiI,euler,euler_p);
% 
% 
% chiy =[   -8.7137   10.0991    1.4996   -1.7877    2.2270   -0.3464   -1.3446];
% chiz = [ 19.8078  -12.8072    1.4176   -3.8751    0.2143    5.6843];
% 
% A = [chi(1) 0 0;
%      0 chiy(1) 0;
%      0 0 chiz(1)];
% B = [chi(2) 0 0;
%      0 chiy(2) 0;
%      0 0 0];
% C = [chi(3) 0 0;
%      0 chiy(3) 0;
%      0 0 chiz(2)];
% D = [chi(4) chi(5) chi(6);
%      chiy(4) chiy(5) chiy(6);
%      chiz(3) chiz(4) chiz(5)];
% E = [chi(7) 0 0;
%      0 chiy(7) 0;
%      0 0 chiz(6)];


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


euler_pp = pinv(Mbar+E)*(A*T_ref + B*euler + C*euler_p - Cbar*euler_p + D*sigma);

end