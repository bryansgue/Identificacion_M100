function [cost] = funcion_costo_full(chi,u_ref,euler,v,v_p,N)

he = [];
for k=1:N
    
    v_p_est(:,k) = Model_v(chi,u_ref(:,k),euler(:,k), v(:,k));
    
    he = [he; v_p(:,k) -  v_p_est(:,k) ];
end
 cost = norm(he,2);% + 0.05*norm(x,1);
%cost = norm(he'*he,2); %;
%cost = he'*he;
%cost = norm(he,'fro')^2; %+  alpha*norm(A, 1) + alpha*norm(B, 1) +  alpha*norm(C, 1) + alpha*norm(D, 1);
end

