function [T] = Model_euler_pp(chi,u_ref,euler,euler_p)


phi_ref = u_ref(1);
theta_ref = u_ref(2);
psi_p_ref = u_ref(3);

phi = euler(1);
theta = euler(2);
psi = euler(3);

phi_p = euler_p(1);
theta_p = euler_p(2);
psi_p = euler_p(3);

c_1 = phi_p*theta_p;
c_2 = phi_p*psi_p;
c_3 = theta_p*psi_p;

I_xx = chi(1);
I_yy = chi(2);
I_zz = chi(3);

tau_x = chi(4)*(phi_ref-phi) - chi(5)*phi_p - chi(6)*c_1 - chi(7)*c_2 - chi(8)*c_3;
tau_y = chi(9)*(theta_ref-theta) - chi(10)*theta_p  - chi(11)*c_1 - chi(12)*c_2 - chi(13)*c_3;
tau_z = chi(14)*(psi_p_ref-psi_p) ;


phi_pp = - ((I_yy*I_zz + I_xx*I_yy*sin(theta)^2 - I_yy*I_zz*sin(theta)^2 - I_xx*I_yy*sin(phi)^2*sin(theta)^2 + I_xx*I_zz*sin(phi)^2*sin(theta)^2)*(tau_x - (I_yy*theta_p^2*sin(2*phi))/2 + (I_zz*theta_p^2*sin(2*phi))/2 + I_xx*psi_p*theta_p*cos(theta) - I_yy*psi_p*theta_p*cos(theta) + I_zz*psi_p*theta_p*cos(theta) + I_yy*psi_p^2*cos(phi)*sin(phi)*cos(theta)^2 - I_zz*psi_p^2*cos(phi)*sin(phi)*cos(theta)^2 + 2*I_yy*psi_p*theta_p*cos(phi)^2*cos(theta) - 2*I_zz*psi_p*theta_p*cos(phi)^2*cos(theta)))/(I_xx*I_yy*I_zz*(sin(theta)^2 - 1)) - (sin(theta)*(I_zz + I_yy*cos(phi)^2 - I_zz*cos(phi)^2)*(tau_z + I_xx*phi_p*theta_p*cos(theta) + I_yy*phi_p*theta_p*cos(theta) - I_zz*phi_p*theta_p*cos(theta) - I_xx*psi_p*theta_p*sin(2*theta) + I_yy*psi_p*theta_p*sin(2*theta) - 2*I_yy*phi_p*theta_p*cos(phi)^2*cos(theta) + 2*I_zz*phi_p*theta_p*cos(phi)^2*cos(theta) + I_yy*theta_p^2*cos(phi)*sin(phi)*sin(theta) - I_zz*theta_p^2*cos(phi)*sin(phi)*sin(theta) - 2*I_yy*phi_p*psi_p*cos(phi)*sin(phi)*cos(theta)^2 + 2*I_zz*phi_p*psi_p*cos(phi)*sin(phi)*cos(theta)^2 - 2*I_yy*psi_p*theta_p*cos(phi)^2*cos(theta)*sin(theta) + 2*I_zz*psi_p*theta_p*cos(phi)^2*cos(theta)*sin(theta)))/(I_yy*I_zz*(sin(theta)^2 - 1)) - (cos(phi)*sin(phi)*sin(theta)*(I_yy - I_zz)*(tau_y + (I_xx*psi_p^2*sin(2*theta))/2 - (I_zz*psi_p^2*sin(2*theta))/2 - I_xx*phi_p*psi_p*cos(theta) + I_yy*phi_p*theta_p*sin(2*phi) - I_zz*phi_p*theta_p*sin(2*phi) - I_yy*phi_p*psi_p*cos(2*phi)*cos(theta) + I_zz*phi_p*psi_p*cos(2*phi)*cos(theta) - I_yy*psi_p^2*sin(phi)^2*cos(theta)*sin(theta) + I_zz*psi_p^2*sin(phi)^2*cos(theta)*sin(theta)))/(I_yy*I_zz*cos(theta));
 
theta_pp = -((I_yy*tau_z*sin(2*phi))/2 - (I_zz*tau_z*sin(2*phi))/2 + I_yy^2*psi_p^2*(sin(theta) - sin(theta)^3) - I_yy*tau_y*cos(theta) - I_yy^2*phi_p*psi_p*cos(theta)^2 + I_yy*tau_y*cos(phi)^2*cos(theta) - I_zz*tau_y*cos(phi)^2*cos(theta) - I_xx*I_yy*psi_p^2*(sin(theta) - sin(theta)^3) - I_yy^2*psi_p^2*cos(phi)^2*cos(theta)^2*sin(theta) + I_zz^2*psi_p^2*cos(phi)^2*cos(theta)^2*sin(theta) + I_yy^2*phi_p*psi_p*cos(phi)^2*cos(theta)^2 - I_zz^2*phi_p*psi_p*cos(phi)^2*cos(theta)^2 + I_yy*tau_x*cos(phi)*sin(phi)*sin(theta) - I_zz*tau_x*cos(phi)*sin(phi)*sin(theta) + I_xx*I_yy*phi_p*psi_p*cos(theta)^2 + I_yy*I_zz*phi_p*psi_p*cos(theta)^2 - I_yy^2*phi_p*theta_p*cos(phi)*sin(phi)*cos(theta) + I_zz^2*phi_p*theta_p*cos(phi)*sin(phi)*cos(theta) + I_xx*I_yy*psi_p^2*cos(phi)^2*cos(theta)^2*sin(theta) - I_xx*I_zz*psi_p^2*cos(phi)^2*cos(theta)^2*sin(theta) - I_xx*I_yy*phi_p*psi_p*cos(phi)^2*cos(theta)^2 + I_xx*I_zz*phi_p*psi_p*cos(phi)^2*cos(theta)^2 + I_xx*I_yy*phi_p*theta_p*cos(phi)*sin(phi)*cos(theta) - I_xx*I_zz*phi_p*theta_p*cos(phi)*sin(phi)*cos(theta) + I_yy^2*psi_p*theta_p*cos(phi)*sin(phi)*cos(theta)*sin(theta) - I_zz^2*psi_p*theta_p*cos(phi)*sin(phi)*cos(theta)*sin(theta) - I_xx*I_yy*psi_p*theta_p*cos(phi)*sin(phi)*cos(theta)*sin(theta) + I_xx*I_zz*psi_p*theta_p*cos(phi)*sin(phi)*cos(theta)*sin(theta))/(I_yy*I_zz*cos(theta));

psi_pp = (I_zz*tau_z + I_zz*tau_x*sin(theta) + I_yy*tau_z*cos(phi)^2 - I_zz*tau_z*cos(phi)^2 + I_yy*tau_x*cos(phi)^2*sin(theta) - I_zz*tau_x*cos(phi)^2*sin(theta) + (I_zz^2*psi_p*theta_p*sin(2*theta))/2 - I_zz^2*phi_p*theta_p*cos(theta) - (I_xx*I_zz*psi_p*theta_p*sin(2*theta))/2 + (I_yy*I_zz*psi_p*theta_p*sin(2*theta))/2 - I_yy*tau_y*cos(phi)*sin(phi)*cos(theta) + I_zz*tau_y*cos(phi)*sin(phi)*cos(theta) + I_xx*I_zz*phi_p*theta_p*cos(theta) + I_yy*I_zz*phi_p*theta_p*cos(theta) - I_yy^2*phi_p*theta_p*cos(phi)^2*cos(theta) + I_zz^2*phi_p*theta_p*cos(phi)^2*cos(theta) + I_yy^2*psi_p^2*cos(phi)*sin(phi)*cos(theta)^2*sin(theta) - I_zz^2*psi_p^2*cos(phi)*sin(phi)*cos(theta)^2*sin(theta) - I_yy^2*phi_p*psi_p*cos(phi)*sin(phi)*cos(theta)^2 + I_zz^2*phi_p*psi_p*cos(phi)*sin(phi)*cos(theta)^2 + I_xx*I_yy*phi_p*theta_p*cos(phi)^2*cos(theta) - I_xx*I_zz*phi_p*theta_p*cos(phi)^2*cos(theta) + I_yy^2*psi_p*theta_p*cos(phi)^2*cos(theta)*sin(theta) - I_zz^2*psi_p*theta_p*cos(phi)^2*cos(theta)*sin(theta) - I_xx*I_yy*psi_p^2*cos(phi)*sin(phi)*cos(theta)^2*sin(theta) + I_xx*I_zz*psi_p^2*cos(phi)*sin(phi)*cos(theta)^2*sin(theta) + I_xx*I_yy*phi_p*psi_p*cos(phi)*sin(phi)*cos(theta)^2 - I_xx*I_zz*phi_p*psi_p*cos(phi)*sin(phi)*cos(theta)^2 - I_xx*I_yy*psi_p*theta_p*cos(phi)^2*cos(theta)*sin(theta) + I_xx*I_zz*psi_p*theta_p*cos(phi)^2*cos(theta)*sin(theta))/(I_yy*I_zz*cos(theta)^2);


T = [phi_pp;theta_pp;psi_pp];

end