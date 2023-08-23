function [To] = Rot_angular(euler)
%UNTITLED2 Summary of this function goes here

%% Euler values
phi = euler(1);
theta =euler(2);
psi = euler(3);

%% Matrix X
To = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);...
               0 cos(phi) -sin(phi);...
               0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
    
   
end

