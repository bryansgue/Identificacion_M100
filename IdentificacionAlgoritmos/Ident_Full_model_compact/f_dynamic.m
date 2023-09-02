function [x] = f_dynamic(x, u, chi, Ts)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

k1 = f_dynamics(x, u, chi);
k2 = f_dynamics(x + (Ts/2)*k1, u, chi);
k3 = f_dynamics(x + (Ts/2)*k2, u, chi);
k4 = f_dynamics(x + Ts*k3, u, chi);
x = x + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end
