function [Model] = dynamic_model_for_ident_fullUAV_Newton(X,nu,nu_p)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

Mbar = M_Newton(X);
Cbar = C_Newton(X,nu);

Model = Mbar*nu_p + Cbar*nu;

end

