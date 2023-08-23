function [S] = Q_fuction_Newton(X)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

S = zeros(6,6);
S(1,1) = X(11);
S(2,2) = X(12);
S(3,3) = X(13);
S(4,4) = X(14);
S(5,5) = X(15);
S(6,6) = X(16);
end

