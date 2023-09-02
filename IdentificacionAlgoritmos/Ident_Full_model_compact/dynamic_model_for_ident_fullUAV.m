function [u_ref] = dynamic_model_for_ident_fullUAV(X,q,q_p,q_pp )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

Mbar = M_matrix_bar(X,q);
Cbar = C_matrix_bar(X,q,q_p);
Gbar = G_matrix_bar(X,q);

u_ref = Mbar*q_pp + Cbar*q_p + Gbar;

end

