function [nu_next] = f_dynamic_Newton(X, nuref, nu, Ts)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

k1 = f_dynamics_Newton(nu, nuref, X);
k2 = f_dynamics_Newton(nu + (Ts/2)*k1, nuref, X);
k3 = f_dynamics_Newton(nu + (Ts/2)*k2, nuref, X);
k4 = f_dynamics_Newton(nu + Ts*k3, nuref, X);
nu_next = nu + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end
