function [nu_p_real] = f_dynamics_Newton(nu, nuref, X)

M = M_Newton(X);
C = C_Newton(X,nu);
P = P_fuction_Newton(X);
Q = Q_fuction_Newton(X);
L = L_fuction_Newton(X);

R = Rot_zyx(nu(4:6));
To = Rot_angular(nu(4:6));

To_L = [R*L(1:3,1:3) L(1:3,4:6); L(4:6,1:3) To*L(4:6,4:6)];
Aux = P*nuref-Q*nu;
Aux1 = R*Aux(1:3,1);
Aux2 = To*Aux(4:6,1);
Input_model = [Aux1;Aux2];

nu_p_real = inv(To_L+M)*(Input_model-C*nu);

end