function [x_p] = function_model(x, input, chi)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
x1 = x(1:6);
x2 = x(7:12);

h = x1(1:3);
euler = x1(4:6);

v = x2(1:3);
euler_p = x2(4:6);

u_ref = input(1:3);
T_ref = input(4:6);

m = chi(25);

I = eye(3);
[A1,B1,Q1,G1,A2,B2,C2,D2,E2,G_tras] = Matrices(chi);

R = Rot_zyx(euler);

phi_p = euler_p(1);
theta_p = euler_p(2);
psi_p = euler_p(3);

c_1 = phi_p*theta_p;
c_2 = phi_p*psi_p;
c_3 = theta_p*psi_p;

sigma = [c_1;c_2;c_3];

M_rot = M_matrix(chi,euler);
C_rot = C_matrix(chi,euler,euler_p);


%% Ecuacion de estados

x1_p = x2;

x2_p_a = pinv(m*I+R*Q1)*(R*(A1*u_ref-B1*v+G1)-G_tras);
x2_p_b = pinv(M_rot+E2)*(A2*T_ref + B2*euler + C2*euler_p - C_rot*euler_p + D2*sigma);


x2_p = [x2_p_a;x2_p_b];

x_p = [x1_p; x2_p];

end

