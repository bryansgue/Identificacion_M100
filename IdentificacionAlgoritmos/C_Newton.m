function [C] = C_Newton(X,nu)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

m = X(1);
Ixx = X(2);
Iyy = X(3);
Izz = X(4);

I = zeros(3,3);
I(1,1) = Ixx;
I(2,2) = Iyy;
I(3,3) = Izz;

u = nu(1:3);
w = nu(4:6);
C1 = skew_symmetric_matrix(u);
C2 = skew_symmetric_matrix(I*w);

C = [zeros(3,3) -m*C1;
    zeros(3,3) -C2];                                                                                                                                                                                                                               0;...
end

