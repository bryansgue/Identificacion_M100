function [S] = L_fuction_Newton(X)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

S = zeros(6,6);
S(1,1) = X(17);
S(2,2) = X(18);
S(3,3) = X(19);
S(4,4) = X(20);
S(5,5) = X(21);
S(6,6) = X(22);
end

