function [D] = D_matrix(X)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes her

D1 = zeros(3,3);
D1(1,1) = X(17);
D1(2,2) = X(18);
D1(3,3) = X(19);

D = [D1 zeros(3,3);
    zeros(3,3) zeros(3,3)];


end

