function [xp] = f_dynamics_sindy(h, u, coe)
%UNTITLED2 Summary of this function goes here

q = h(1:6);
q_p = h(7:12);

nx = q(1);
ny = q(2);
nz = q(3);
phi = q(4);
theta = q(5);
psi = q(6);
nx_p = q_p(1);
ny_p = q_p(2);
nz_p = q_p(3);
phi_p = q_p(4);
theta_p = q_p(5);
psi_p = q_p(6);
u1 = u(1);
u2 = u(2);
u3 = u(3);
u4 = u(4);

    X = [
    nx;
    ny;
    nz;
    phi;
    theta;
    psi;
    nx_p;
    ny_p;
    nz_p;
    phi_p;
    theta_p;
    psi_p;
    u1;
    u2;
    u3;
    u4;
    sin(nx);
    cos(nx);
    sin(phi);
    cos(phi);
    sin(theta);
    cos(theta);
    sin(psi);
    cos(psi)];

    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

size(X)
size(coe)

xp =  coe*X;

end

