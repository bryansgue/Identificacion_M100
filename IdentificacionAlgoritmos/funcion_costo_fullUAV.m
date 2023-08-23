function [cost] = funcion_costo_fullUAV(X,u_ref,q,q_p,q_pp,N)

he = [];
for k=1:N
    vref_system(:,k) = dynamic_model_for_ident_fullUAV(X,q(:,k),q_p(:,k),q_pp(:,k));
    Input_model(:,k) = Model_input_fullUAV(X,u_ref(:,k), q(:,k), q_p(:,k),q_pp(:,k));
    
    R = Rot_zyx(q(4:6,k));
    aux_1 = R*Input_model(1:3,k);
    aux_2 = Input_model(4:6,k);
    
    he = [he; vref_system(:,k) - [aux_1;aux_2]];
end
cost = norm(he,2);% + 0.05*norm(x,1);
%cost = norm(he'*he,2); %;
%cost = he'*he;
end

