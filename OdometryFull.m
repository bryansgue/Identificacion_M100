function [euler_B, omega_B, linear_aceleration_B, velocity_I, position_I] = OdometryFull(odomSub)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Read odometry values from ros
odomdata = receive(odomSub,0.2);  %(the second argument is a time-out in seconds).

data = odomdata.Axes;

euler_B = data(1:3);
omega_B = data(4:6);
linear_aceleration_B = data(7:9);
velocity_I = data(10:12);
position_I = data(13:15);
end

