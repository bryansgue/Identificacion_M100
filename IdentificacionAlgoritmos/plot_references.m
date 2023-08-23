function plot_references(t, u_ref)
% Función para graficar vectores de señales de referencia
% t: vector de tiempo
% u_ref: matriz de señales de referencia (6 x n), donde n es el número de muestras

F_ref = u_ref(3,:);
phi_ref = u_ref(4,:);
theta_ref = u_ref(5,:);
psi_ref = u_ref(6,:);

% Graficar señal de fuerza de empuje
subplot(2,2,1)
plot(t, F_ref)
title('Fuerza de empuje')
xlabel('Tiempo [s]')
ylabel('Fuerza [N]')

% Graficar señal de ángulo de balanceo
subplot(2,2,2)
plot(t, phi_ref)
title('Ángulo de balanceo')
xlabel('Tiempo [s]')
ylabel('Ángulo [rad]')

% Graficar señal de ángulo de cabeceo
subplot(2,2,3)
plot(t, theta_ref)
title('Ángulo de cabeceo')
xlabel('Tiempo [s]')
ylabel('Ángulo [rad]')

% Graficar señal de ángulo de guiñada
subplot(2,2,4)
plot(t, psi_ref)
title('Ángulo de guiñada')
xlabel('Tiempo [s]')
ylabel('Ángulo [rad]')

end