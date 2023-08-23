function [cost] = funcion_costo(chi,u_ref,euler,euler_p, euler_pp,N)

he = [];
for k=1:N
    
    [euler_pp_est(:,k),A,B,C,D] = Model_euler_pp(chi,u_ref(:,k),euler(:,k),euler_p(:,k));
    
    he = [he; euler_pp(:,k) -  euler_pp_est(:,k) ];
    
end
 cost = norm(he,2);% + 0.05*norm(x,1);
%cost = norm(he'*he,2); %;
%cost = he'*he;
%cost = norm(he,"fro")^2; %+  alpha*norm(A, 1) + alpha*norm(B, 1) +  alpha*norm(C, 1) + alpha*norm(D, 1);
end

