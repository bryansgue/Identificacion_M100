function [x] = RK4_euler_uav(x, u, chi, ts)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

k1 = function_euler(x, u, chi);
k2 = function_euler(x + (ts/2)*k1, u, chi);
k3 = function_euler(x + (ts/2)*k2, u, chi);
k4 = function_euler(x + ts*k3, u, chi);
x = x + (ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end
