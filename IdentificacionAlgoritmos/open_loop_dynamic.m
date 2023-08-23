function [vref] = open_loop_dynamic(x, vp, v, N, psi, L)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
for k=1:N
   vref(:,k) = dynamic_identification(x, vp(:,k), v(:,k), psi(k), L); 
end
end

