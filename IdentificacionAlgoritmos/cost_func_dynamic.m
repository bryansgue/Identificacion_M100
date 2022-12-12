function [cost] = cost_func_dynamic(x, vref, vp, v, N, psi, L)
%UNTITLED6 Summary of this function goes here
vref_system = open_loop_dynamic(x, vp, v, N, psi, L);
he = error_dynamic(vref, vref_system, N);
cost = he'*he;
end


