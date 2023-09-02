function v_p = Model_v(chi,u_ref,euler, v)


g = 9.81;
m = chi(1);

I = eye(3);
A = [0 0 0;
    0 0 0
    0 0 chi(2)];
B = [0 0 0;
    0 0 0
    0 0 chi(3)];
Q = [0 0 0;
    0 0 0
    0 0 chi(4)];

G = 1*[0;0;m*g];
Gbar = 1*[0;0;m*g];

R = Rot_zyx(euler);

v_p = pinv(m*I+R*Q)*(R*(A*u_ref-B*v+G)-Gbar);


end