function [vref] = dynamic_model_for_ident(x, vp, v, omega, omega_p,L)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

%% EXTRACCION OF GENERALIZED VECTOR

w = omega(3);
a = L(1);
b = L(2);
% INERCIAL MATRIX
M11=x(1);
M12=0;
M13=0;
M14=a*w*x(2);
M21=0;
M22=x(3) ;
M23=0;
M24=b*w*x(4);
M31=0;
M32=0;
M33=x(5);
M34=0;
M41=a*w*x(6);
M42=b*w*x(7);
M43=0;
M44=x(8);



M=[M11,M12,M13,M14;...
    M21,M22,M23,M24;...
    M31,M32,M33,M34;...
    M41,M42,M43,M44];

%% CENTRIOLIS MATRIX
C11=x(9);
C12=0;
C13=0;
C14=a*w*x(10);
C21=0;
C22=x(11);
C23=0;
C24=b*w*x(12);
C31=0;
C32=0;
C33=x(13);
C34=0;
C41=b*(w^2)*x(14);
C42=a*(w^2)*x(15);
C43=0;
C44=(a^2)*(w^2)*x(16)+(b^2)*(w^2)*x(17)+x(18);

C=[C11,C12,C13,C14;...
    C21,C22,C23,C24;...
    C31,C32,C33,C34;...
    C41,C42,C43,C44];

%% GRAVITATIONAL MATRIX
G11=0;
G21=0;
G31=0;
G41=0;

G=[G11;G21;G31;G41];

vref = M*[vp;omega_p(3)] + C*[v;omega(3)] + G;
end

