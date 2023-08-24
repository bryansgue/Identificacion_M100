function [cost] = funcion_costo(x, u_ref, u_p, u, omega,omega_p, N, L)

he = [];
for k=1:N
    vref_system(:,k) = dynamic_model_for_ident(x, u_p(:,k), u(:,k), omega(:,k),omega_p(:,k), L);
    
    he = [he; u_ref(:,k)-vref_system(:,k)];
end
cost = norm(he,2);% + 0.05*norm(x,1);
%cost = norm(he'*he,2); %;
%cost = he'*he;
end

