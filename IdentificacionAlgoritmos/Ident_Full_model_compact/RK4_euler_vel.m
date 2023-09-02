function [x] = RK4_euler_vel(x, u, chi, ts, euler)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

k1 = function_vel(x, u, chi, euler);
k2 = function_vel(x + (ts/2)*k1, u, chi, euler);
k3 = function_vel(x + (ts/2)*k2, u, chi, euler);
k4 = function_vel(x + ts*k3, u, chi, euler);
x = x + (ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end
