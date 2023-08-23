function [S] = P_fuction_Newton(X)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

S = zeros(6,6);
S(1,1) = X(5);
S(2,2) = X(6);
S(3,3) = X(7);
S(4,4) = X(8);
S(5,5) = X(9);
S(6,6) = X(10);

end

