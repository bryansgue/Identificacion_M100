function [cost] = funcion_costo(x, vref, vp, v, N, psi, L)

he = [];
for k=1:N
    vref_system(:,k) = dynamic_model_for_ident(x, vp(:,k), v(:,k), psi(k), L);
    he = [he; vref(:,k)-vref_system(:,k)];
end
cost = norm(he,1);% + 0.05*norm(x,1);
%cost = norm(he'*he,2); %;
%cost = he'*he;
end

