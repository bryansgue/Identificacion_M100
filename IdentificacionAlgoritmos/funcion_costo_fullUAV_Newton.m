function [cost] = funcion_costo_fullUAV_Newton(X,nu_ref,nu,nu_p,N)

he = [];
for k=1:N
    vref_system(:,k) = dynamic_model_for_ident_fullUAV_Newton(X,nu(:,k),nu_p(:,k));
    Input_model(:,k) = Model_input_fullUAV_Newton(X,nu_ref(:,k), nu(:,k), nu_p(:,k));
    
    R = Rot_zyx(nu(4:6,k));
    To = Rot_angular(nu(4:6,k));
    
    aux_1 = R*Input_model(1:3,k);
    aux_2 = To*Input_model(4:6,k);
    
    he = [he; vref_system(:,k) - [aux_1;aux_2] ];
end
cost = norm(he,2);% + 0.05*norm(x,1);
%cost = norm(he'*he,2); %;
%cost = he'*he;
end

