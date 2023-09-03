function [u_real] = Model_input_fullUAV(X,u_ref, q, q_p, q_pp)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

g = 9.81;
m = X(1);

 S = S_fuction(X);
 Q = Q_fuction(X);
 E = E_fuction(X);
 T = T_fuction(X);
 
 G = [0;0;m*g;0;0;0];

u_real = S*u_ref -Q*q -E*q_p -T*q_pp+G;

end