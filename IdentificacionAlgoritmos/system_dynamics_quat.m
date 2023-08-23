function [xp] = system_dynamics_quat(x, uref, chi)
%UNTITLED2 Summary of this function goes here
%% State of the system
h = x(1:3);
v = x(4:6);
quat = x(7:10);
omega = x(11:13);

%   Detailed explanation goes here

ul_ref = uref(1);
um_ref = uref(2);
un_ref = uref(3);
w_ref = uref(4);



wz = omega(3);



%% System parameters
g = 9.8;
m = chi(1);
Jxx = chi(2);
Jyy = chi(3);
Jzz = chi(4);

%% Inertia Mmatrix
I = [Jxx, 0 0;...
     0, Jyy, 0;...
     0, 0, Jzz];
 
%% Torques inputs

e = [0;0;1];


%% Rotational matrix
[R] = QuatToRot(quat);

%% Force body frame Drone


Kp_ref = [chi(5) 0 0;
          0 chi(5) 0;
          0 0 chi(5)];

Kp_v = [chi(6) 0 0;
        0 chi(6) 0;
        0 0 chi(6)];

Kd_vp = [chi(7) 0 0;
        0 chi(7) 0;
        0 0 chi(7)];

E = [0 0 0;
     0 0 0;
     0 0 1];

A = [chi(8) 0 0;
     0 chi(11) 0;
     0 0 chi(14)];
B = [chi(9) 0 0;
     0 chi(12) 0;
     0 0 0];
C = [0 0 0;
     0 0 0;
     0 0 chi(15)];
D = [chi(10) 0 0;
     0 chi(13) 0;
     0 0 0];
Q = [0 0 0;
     0 0 0;
     0 0 chi(16)];

l = [ul_ref;um_ref;w_ref];

%% Aceleration system 

v_p = inv(I+ 1/m * E * Kd_vp)*[ -e*g + 1/m *E*R*Kp_ref*uref(1:3) -  1/m *E*Kp_v*v];

q_dot = quat_p(quat, omega);

omega_p = inv(I+Q)*(A*l-B*inv(R)*v-C*omega-D*inv(R)*v_p - cross(omega, I*omega));


%% General vector of the system
xp = zeros(13, 1);
xp(1:3) = v;
xp(4:6) = v_p;
xp(7:10) = q_dot;
xp(11:13) = omega_p;
end

