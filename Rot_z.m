function [R] = Rot_z(euler)
%UNTITLED2 Summary of this function goes here

%% Euler values

psi = euler(3);

%% Matrix X

    
RotZ = [cos(psi) -sin(psi) 0;...
        sin(psi) cos(psi) 0;...
        0 0 1];

R = RotZ;
   
end

