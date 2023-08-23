function [M] = M_Newton(X)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
m = X(1);
Ixx = X(2);
Iyy = X(3);
Izz = X(4);

M = zeros(6,6);
M(1,1) = m;
M(2,2) = m;
M(3,3) = m;
M(4,4) = Ixx;
M(5,5) = Iyy;
M(6,6) = Izz;
end

