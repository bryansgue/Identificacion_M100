function [q_dot] =quat_p(quat, omega)
%Derivative of quaternions
%   This function gets the derivate of the quaternions

q_dot = (1/2)* quaternionMultiply(quat, [0;omega]);
end

