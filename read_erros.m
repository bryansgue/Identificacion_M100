function [values] = read_erros(odomSub)
% Read odometry values from ros
odomdata = receive(odomSub,0.2);  %(the second argument is a time-out in seconds).

vel_lineal = odomdata.Twist.Linear;
vel_angular = odomdata.Twist.Angular;

values = [vel_lineal.x; vel_lineal.y; vel_lineal.z; vel_angular.x]; 
end
