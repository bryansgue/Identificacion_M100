function [f_system] = dynamic_model_for_ident_fullUAV_quat(x, uref, u, u_p, v, v_p, omega, omega_p, quat, R)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

ul_ref = uref(1);
um_ref = uref(2);
un_ref = uref(3);
w_ref = uref(4);

ul = u(1);
um = u(2);
un = u(3);

ul_p = u_p(1);
um_p = u_p(2);
un_p = u_p(3);

wz = omega(3);
wz_p = omega_p(3);

%% System parameters
g = 9.8;
m = x(1);
Jxx = x(2);
Jyy = x(3);
Jzz = x(4);

%% Inertia Mmatrix
I = [Jxx, 0 0;...
     0, Jyy, 0;...
     0, 0, Jzz];
 
%% Torques inputs

e = [0;0;1];


%% Rotational matrix
%[R] = quaternionToRotationMatrix(quat);

%% Force body frame Drone
u_T =     x(5) * un_ref- x(6) * un - x(7) * un_p;
u_theta = x(8) * ul_ref- x(9) * ul - x(10)* ul_p;
u_phi =   x(11)* um_ref- x(12)* um - x(13)* um_p;
u_psi =   x(14)* w_ref - x(15)* wz - x(16)* wz_p;

T = [0; 0; u_T];
Tau = [u_theta; u_phi; u_psi];

%% Aceleration system 
v_p = -e*g + (R * T)/m;

% q_dot = quat_p(quat, omega);

omega_p = inv(I)*(Tau - cross(omega, I*omega));


f_system = [v_p;omega_p];
%% General vector of the system

end