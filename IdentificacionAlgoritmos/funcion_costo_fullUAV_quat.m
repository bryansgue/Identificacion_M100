function [cost] = funcion_costo_fullUAV_quat(x, uref, u, u_p, v, v_p, omega, omega_p, quat,R, N)
                                             
he = [];
for k=1:N-1

    f_system(:,k) = dynamic_model_for_ident_fullUAV_quat(x, uref(:,k), u(:,k), u_p(:,k), v(:,k), v_p(:,k), omega(:,k), omega_p(:,k), quat(:,k),R(:,:,k));
    
    he = [he; f_system(:,k) - [v_p(:,k);omega_p(:,k)] ];
end
cost = norm(he,2);% + 0.05*norm(x,1);
%cost = norm(he'*he,2); %;
%cost = he'*he;
end

