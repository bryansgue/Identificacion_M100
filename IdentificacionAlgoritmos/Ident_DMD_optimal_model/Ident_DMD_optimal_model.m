
%% IDENTIFICATION OF PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
n=4;
n_chr = int2str(n);
text1 = 'states_';
text2 = 'u_ref_';
text3 = 't_';
extension = '.mat';

load(strcat(text1,n_chr,extension))
load(strcat(text2,n_chr,extension))
load(strcat(text3,n_chr,extension))

%%
% load('states_1.mat')
% load('T_ref_1.mat')
% load('t_1.mat')

%%
pose = states(1:3,:);
euler = states(4:6,:);
vel = states(7:9,:);
euler_p = states(10:12,:);
omega = states(13:15,:);
quat = states(16:19,:);
 u = states(20:22,:);

%% Settings
clear tf;

final = 0;
init = 6;
t = t(1,1:end-final);

dim = length(t);

t = t(1,init:end);
N = length(t);

ts = 1/30;

%% REFERENCE SIGNALS 

ul_ref = u_ref(1,init:dim);
um_ref = u_ref(2,init:dim);
un_ref = u_ref(3,init:dim);
r_ref = u_ref(4,init:dim);

% REFERENCE SIGNALS

u_ref = [ul_ref; um_ref; un_ref; r_ref];

%% POSICIONES
hx = double(pose(1,init:dim));
hy = double(pose(2,init:dim));
hz = double(pose(3,init:dim));

h =  [hx;hy;hz];

%% EULER
phi = double(euler(1,init:dim));
theta = double(euler(2,init:dim));
psi = double(euler(3,init:dim));

eul = [phi;theta; psi];

%% EULER_P
phi_p = double(euler_p(1,init:dim));
theta_p = double(euler_p(2,init:dim));
psi_p = double(euler_p(3,init:dim));

eul_p =  [phi_p;theta_p; psi_p];

%% OMEGA
p = double(omega(1,init:dim));
q = double(omega(2,init:dim));
r = double(omega(3,init:dim));

omega =  [p;q; r];

%% VELOCIDAD INERTIAL
vx = double(vel(1,init:dim));
vy = double(vel(2,init:dim));
vz = double(vel(3,init:dim));

v =  [vx; vy; vz];

%% VELOCIDAD BODY

% for k=1:length(eul)
%      R = Rot_z(eul(:,k));
%      u(:,k) = pinv(R)*v(:,k);
% end 

% ul = double(u(1,:));
% um = double(u(2,:));
% un = double(u(3,:));
 


 ul = double(u(1,init:dim));
 um = double(u(2,init:dim));
 un = double(u(3,init:dim));

u =  [ul; um; un];

%% REAL SYSTEM ACCELERATIONS
phi_pp = [0, diff(phi_p)/ts];
theta_pp = [0 , diff(theta_p)/ts];
psi_pp = [0 , diff(psi_p)/ts];

eul_pp =  [phi_pp;theta_pp; psi_pp];

%% REAL SYSTEM Linea; ACCELERATIONS
vx_p = [0, diff(vx)/ts];
vy_p = [0 , diff(vy)/ts];
vz_p = [0 , diff(vz)/ts];

v_p =  [vx_p; vy_p; vz_p];

%% REAL SYSTEM Linea; ACCELERATIONS
ul_p = [0, diff(ul)/ts];
um_p = [0 , diff(um)/ts];
un_p = [0 , diff(un)/ts];

u_p =  [ul_p; um_p; un_p];

%% REAL SYSTEM Linea; ACCELERATIONS
p_p = [0, diff(p)/ts];
q_p = [0 , diff(q)/ts];
r_p = [0 , diff(r)/ts];

omega_p = [p_p; q_p; r_p];

%% ESTADOS
x = [ul; um; un; r];
x_p = [ul_p; um_p; un_p; r_p];

%% Filter signals
landa = 25;
F1=tf(landa,[1 landa]);

%% REFERENCE SIGNALS Filtradas
u_ref_f(1,:) = lsim(F1,u_ref(1,:),t);
u_ref_f(2,:) = lsim(F1,u_ref(2,:),t);
u_ref_f(3,:) = lsim(F1,u_ref(3,:),t);
u_ref_f(4,:) = lsim(F1,u_ref(4,:),t);


%% REAL SYSTEM Filter
h_f(1,:) = lsim(F1,h(1,:),t);
h_f(2,:) = lsim(F1,h(2,:),t);
h_f(3,:) = lsim(F1,h(3,:),t);

%% REAL SYSTEM Filter
eul_f(1,:) = lsim(F1,eul(1,:),t);
eul_f(2,:) = lsim(F1,eul(2,:),t);
eul_f(3,:) = lsim(F1,eul(3,:),t);

omega_f(1,:) = lsim(F1,omega(1,:),t);
omega_f(2,:) = lsim(F1,omega(2,:),t);
omega_f(3,:) = lsim(F1,omega(3,:),t);

omega_p_f(1,:) = lsim(F1,omega_p(1,:),t);
omega_p_f(2,:) = lsim(F1,omega_p(2,:),t);
omega_p_f(3,:) = lsim(F1,omega_p(3,:),t);


%% REAL SYSTEM_P Filter
eul_p_f(1,:) = lsim(F1,eul_p(1,:),t);
eul_p_f(2,:) = lsim(F1,eul_p(2,:),t);
eul_p_f(3,:) = lsim(F1,eul_p(3,:),t);

v_f(1,:) = lsim(F1,v(1,:),t);
v_f(2,:) = lsim(F1,v(2,:),t);
v_f(3,:) = lsim(F1,v(3,:),t);

u_f(1,:) = lsim(F1,u(1,:),t);
u_f(2,:) = lsim(F1,u(2,:),t);
u_f(3,:) = lsim(F1,u(3,:),t);

u_p_f(1,:) = lsim(F1,u_p(1,:),t);
u_p_f(2,:) = lsim(F1,u_p(2,:),t);
u_p_f(3,:) = lsim(F1,u_p(3,:),t);

v_p_f(1,:) = lsim(F1,v_p(1,:),t);
v_p_f(2,:) = lsim(F1,v_p(2,:),t);
v_p_f(3,:) = lsim(F1,v_p(3,:),t);


%% REAL SYSTEM_PP Filter
eul_pp_f(1,:) = lsim(F1,eul_pp(1,:),t);
eul_pp_f(2,:) = lsim(F1,eul_pp(2,:),t);
eul_pp_f(3,:) = lsim(F1,eul_pp(3,:),t);


%% Parameter matrices
a = 0;
b = 0;
L = [0;0];

%
%% CALCULO DEL MODELO
u_f = [u_f;omega_f(3,:)];
u_p_f = [u_p_f;omega_p_f(3,:)];

%% Parametros del optimizador
% options = optimset('Display','iter',...
%     'TolFun', 1e-8,...
%     'MaxIter', 60000,...
%     'MaxFunEvals', 10000,... % Agregado nuevo_valor aquí
%     'Algorithm', 'active-set',...
%     'FinDiffType', 'forward',...
%     'RelLineSrchBnd', [],...
%     'RelLineSrchBndDuration', 1,...
%     'TolConSQP', 2e-8);
% 
% x0 = ones(1,19);
% %x0 = [ones(1,3),chix, chiy,chiz];
% f_obj1 = @(x) funcion_costo(x, u_ref_f, u_p_f, u_f, omega_f,omega_p_f, N, L); 
%                             
% chi = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
%  %
 
%%

options = optimset('Display','iter',...
    'TolFun', 1e-8,...
    'MaxIter', 60000,...
    'MaxFunEvals', 10000,...
    'Algorithm', 'active-set',...
    'FinDiffType', 'forward',...
    'RelLineSrchBnd', [],...
    'RelLineSrchBndDuration', 1,...
    'TolConSQP', 2e-8);

% Número de puntos iniciales aleatorios que deseas generar
num_starts = 1;

% Inicializa un arreglo para almacenar los resultados de cada optimización
results = cell(num_starts, 1);

for i = 1:num_starts
    x0 = ones(1,32) .* rand(1,32);
%     lb = zeros(1,16);
                                   
    f_obj1 = @(x) funcion_costo(x, u_ref_f, u_p_f, u_f, N);                                 
    results{i} = fmincon(f_obj1, x0, [], [], [], [], [], [], [], options);
end

% Encuentra la mejor solución y su costo utilizando min
[best_cost, best_idx] = min(cellfun(@(x) f_obj1(x), results));
chi = results{best_idx};

%%
A = [chi(1) chi(2) chi(3) chi(4);
    chi(5) chi(6) chi(7) chi(8);
    chi(9) chi(10) chi(11) chi(12);
    chi(13) chi(14) chi(15) chi(16)]

B = [chi(17) chi(18) chi(19) chi(20);
    chi(21) chi(22) chi(23) chi(24);
    chi(25) chi(26) chi(27) chi(28);
    chi(29) chi(30) chi(31) chi(32)]

% A = zeros(4,4);
% A(1,1) = chi(1);
% A(2,2) = chi(2);
% A(3,3) = chi(3);
% A(4,4) = chi(4);
% 
% B = zeros(4,4);
% B(1,1) = chi(5);
% B(2,2) = chi(6);
% B(3,3) = chi(7);
% B(4,4) = chi(8);


%%


% Determinar los valores mínimos y máximos de la matriz
valor_minimo = min(A(:));
valor_maximo = max(A(:));

% Realizar una expansión de contraste
matriz_expandida = (A - valor_minimo) / (valor_maximo - valor_minimo);

% Crear una nueva figura con dimensiones personalizadas
figure('Position', [100, 100, 800, 800]);

% Mostrar la matriz expandida como una imagen con un tamaño personalizado
imshow(matriz_expandida, 'InitialMagnification', 'fit');
title('Matriz con contraste expandido');

% Ajustar los ejes si es necesario
axis on; % Muestra los ejes
% Especificar una paleta de colores (en este caso, jet)
%colormap(blue(256)); % Puedes cambiar 'jet' a otra paleta de colores si lo deseas
colorbar; % Agrega una barra de colores para mostrar la escala de valores

%%
%% SIMULATION DYNAMICS
u_estimate = u_f(:,1);
for k=1:length(t)
    a = (A*u_estimate(:,k)+B*u_ref_f(:,k));
    u_estimate(:, k+1) = u_estimate(:, k) + a*ts;
end

%save("IdentDMD_test.mat","v_estimate","v_ref","v_real","t");
% save("A_B_values_simulado.mat","A","B");
%%
% Primer subplot: x_estimate y phi_p
subplot(4,1,1)
plot(x(1,:), 'LineWidth', 2, 'DisplayName', 'ul_{real}');
hold on
plot(u_estimate(1,:), 'LineWidth', 2, 'DisplayName', 'ul_{estimate}');
hold on
plot(u_ref(1,:) ,'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1, 'DisplayName', 'ul_{ref}');
ylabel('Valores');
title('Comparación de phi\_estimate y phi\_p');
legend;
grid on

% Segundo subplot: x_estimate y theta_p
subplot(4,1,2)
plot(x(2,:), 'LineWidth', 2, 'DisplayName', 'um_{real}');
hold on
plot(u_estimate(2,:), 'LineWidth', 2, 'DisplayName', 'um_{estimate}');
hold on
plot(u_ref(2,:),'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1, 'DisplayName', 'um_{ref}');
ylabel('Valores');
title('Comparación de theta\_estimate y theta\_p');
legend;
grid on

% Tercer subplot: x_estimate y psi_p
subplot(4,1,3)


plot(x(3,:), 'LineWidth', 2, 'DisplayName', 'ul_{real}');
hold on
plot(u_estimate(3,:), 'LineWidth', 2, 'DisplayName', 'ul_{estimate}');
hold on
plot(u_ref(3,:),'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1, 'DisplayName', 'ul_{ref}');
xlabel('Tiempo');
ylabel('Valores');
title('Comparación de psi\_estimate y psi\_p');
legend;
grid on

% Tercer subplot: x_estimate y psi_p
subplot(4,1,4)
plot(x(4,:), 'LineWidth', 2, 'DisplayName', 'r_{real}');
hold on
plot(u_estimate(4,:), 'LineWidth', 2, 'DisplayName', 'r_{estimate}');
hold on
plot(u_ref(4,:),'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1, 'DisplayName', 'r_{ref}');

xlabel('Tiempo');
ylabel('Valores');
title('Comparación de psi\_estimate y psi\_p');
legend;
grid on

