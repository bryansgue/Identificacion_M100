function [T_ref] = model_for_ident_fullUAV(chi,euler,euler_p,euler_pp )

chiI =    1.0e-06 * [ -0.0406 -0.2634 0.2445];

M = M_matrix(chiI,euler);
C = C_matrix(chiI,euler,euler_p);

T_ref = M*euler_pp + C*euler_p;

end

