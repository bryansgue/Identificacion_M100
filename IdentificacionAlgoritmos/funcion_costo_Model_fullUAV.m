function [cost] = funcion_costo_Model_fullUAV(chi,u_ref,euler,euler_p,euler_pp,N)

he = [];
for k=1:N
    T_system(:,k) = model_for_ident_fullUAV(chi,euler(:,k),euler_p(:,k),euler_pp(:,k));
    input_model(:,k) = ModelPD_input_fullUAV(chi,u_ref(:,k), euler(:,k), euler_p(:,k),euler_pp(:,k));
       
    he = [he; T_system(:,k) - input_model(:,k)];
end
cost = norm(he,2);% + 0.05*norm(x,1);
%cost = norm(he'*he,2); %;
%cost = he'*he;
end

