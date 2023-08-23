function [cost] = cost_func_ident_fullUAV(chi,u_ref,euler,euler_p, euler_pp,N)

he = [];
for k=1:N
    
    euler_pp_est(:,k) = Model_euler_pp(chi,u_ref(:,k),euler(:,k),euler_p(:,k));
    
    he = [he; euler_pp(:,k) -  euler_pp_est(:,k) ];
end
cost = norm(he,2);% + 0.05*norm(x,1);
%cost = norm(he'*he,2); %;
%cost = he'*he;
end

