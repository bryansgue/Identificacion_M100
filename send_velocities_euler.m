function  send_velocities_euler(robot, velmsg, vd)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Send desired velocities to the robot Linear
F = vd(1);
Tx = vd(2);
Ty = vd(3);


% Send desired velocities to the robot angular
wx = 0;
wy = 0;
Tz = vd(4);

velmsg.Twist.Linear.X = 0;
velmsg.Twist.Linear.Y = 0;
velmsg.Twist.Linear.Z = F;

velmsg.Twist.Angular.X = Tx;
velmsg.Twist.Angular.Y = Ty;
velmsg.Twist.Angular.Z = Tz;


send(robot,velmsg);
end

