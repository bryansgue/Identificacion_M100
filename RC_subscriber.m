function [hd] = RC_subscriber(RcSub)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


rc_data = receive(RcSub,1); 


hd = rc_data.Axes;



end

