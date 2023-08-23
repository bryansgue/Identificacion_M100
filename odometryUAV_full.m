function [position,v_lineal,v_angular,quaternio, euler] = odometryUAV_full(odomSub)
% Read odometry values from ros
odomdata = receive(odomSub,0.2);  %(the second argument is a time-out in seconds).

pose = odomdata.Pose.Pose.Position;
vel_lineal = odomdata.Twist.Twist.Linear;
vel_angular = odomdata.Twist.Twist.Angular;
quat = odomdata.Pose.Pose.Orientation;

position = [pose.X;pose.Y;pose.Z];
v_lineal = [vel_lineal.X;vel_lineal.Y;vel_lineal.Z];
v_angular = [vel_angular.X;vel_angular.Y;vel_angular.Z];
quaternio = [quat.W; quat.X; quat.Y; quat.Z]';

euler_aux = quat2eul(quaternio, 'XYZ');
euler = [euler_aux(2);-euler_aux(1);euler_aux(3)];


end
