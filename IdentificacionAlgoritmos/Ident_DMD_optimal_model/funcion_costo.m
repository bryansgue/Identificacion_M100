function [cost] = funcion_costo(x, u_ref, u_p, u, N)

he = [];
for k=1:N
    
    [u_p_model(:,k), A, B] = DMD_model_ident(x,u(:,k),u_ref(:,k));
    
    he = [he; u_p(:,k)-u_p_model(:,k)];
end


gamma = 5;
cost = norm(he,2) + 2*norm(A,1) + gamma*norm(B,1) ;
%cost = norm(he'*he,2); %;
%cost = he'*he;
end

