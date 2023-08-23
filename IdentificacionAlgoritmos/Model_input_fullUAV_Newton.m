function [F_real] = Model_input_fullUAV_Newton(X,nu_ref, nu, nu_p)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

 P = P_fuction_Newton(X);
 Q = Q_fuction_Newton(X);
 L = L_fuction_Newton(X);
 
% A = A_fuction(X);
% B = B_fuction(X);
F_real = P*nu_ref-Q*nu-L*nu_p;
%u_real = A*q_p+B*u_ref;
end