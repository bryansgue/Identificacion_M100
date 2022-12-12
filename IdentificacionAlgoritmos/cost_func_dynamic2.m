function [cost] = cost_func_dynamic2(x, vref, vp, v, N, psi, L)
%UNTITLED6 Summary of this function goes here
%vref_system = open_loop_dynamic(x, vp, v, N, psi, L);

for k=1:N
   vref_system(:,k) = dynamic_model_for_ident(x, vp(:,k), v(:,k), psi(k), L); 
end

he = error_dynamic(vref, vref_system, N);
cost = he'*he;
end


