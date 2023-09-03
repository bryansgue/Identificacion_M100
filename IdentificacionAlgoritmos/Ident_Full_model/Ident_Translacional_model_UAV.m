
%% IDENTIFICATION OF PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
n=2;
n_chr = int2str(n);
text1 = 'states_';
text2 = 'T_ref_';
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
init = 20;
t = t(1,1:end-final);

dim = length(t);

t = t(1,init:end);
N = length(t);

ts = 0.03;

%% REFERENCE SIGNALS 

vz_ref = T_ref(1,init:dim);
phi_ref = T_ref(2,init:dim);
theta_ref = T_ref(3,init:dim);
psi_p_ref = T_ref(4,init:dim);

phi_ref_max = max(abs(phi_ref));
theta_ref_max = max(abs(theta_ref));
psi_p_ref_max = max(abs(psi_p_ref));

% REFERENCE SIGNALS
u_ref = [0*vz_ref;0*vz_ref;vz_ref];
T_ref = [phi_ref; theta_ref; psi_p_ref];

%% REAL SYSTEM states
hx = double(pose(1,init:dim));
hy = double(pose(2,init:dim));
hz = double(pose(3,init:dim));

h =  [hx;hy;hz];

%%
phi = double(euler(1,init:dim));
theta = double(euler(2,init:dim));
psi = double(euler(3,init:dim));

phi_max = max(abs(phi));
theta_max = max(abs(theta));
psi_max = max(abs(psi));

eul = [phi;theta; psi];

%% REAL SYSTEM State_p
phi_p = double(euler_p(1,init:dim));
theta_p = double(euler_p(2,init:dim));
psi_p = double(euler_p(3,init:dim));

phi_p_max = max(abs(phi_p));
theta_p_max = max(abs(theta_p));
psi_p_max = max(abs(psi_p));

eul_p =  [phi_p;theta_p; psi_p];


%% REAL Velocity
vx = double(vel(1,init:dim));
vy = double(vel(2,init:dim));
vz = double(vel(3,init:dim));

vx_max = max(abs(vx));
vy_max = max(abs(vy));
vz_max = max(abs(vz));

v =  [vx;vy; vz];

%% REAL SYSTEM ACCELERATIONS
phi_pp = [0, diff(phi_p)/ts];
theta_pp = [0 , diff(theta_p)/ts];
psi_pp = [0 , diff(psi_p)/ts];

phi_pp_max = max(abs(phi_pp));
theta_pp_max = max(abs(theta_pp));
psi_pp_max = max(abs(psi_pp));

eul_pp =  [phi_pp;theta_pp; psi_pp];

%% REAL SYSTEM Linea; ACCELERATIONS
vx_p = [0, diff(vx)/ts];
vy_p = [0 , diff(vy)/ts];
vz_p = [0 , diff(vz)/ts];

v_p =  [vx_p; vy_p; vz_p];

%% Filter signals
landa = 25;
F1=tf(landa,[1 landa]);

%% REFERENCE SIGNALS Filtradas
u_ref_f(1,:) = lsim(F1,u_ref(1,:),t);
u_ref_f(2,:) = lsim(F1,u_ref(2,:),t);
u_ref_f(3,:) = lsim(F1,u_ref(3,:),t);


T_ref_f(1,:) = lsim(F1,T_ref(1,:),t);
T_ref_f(2,:) = lsim(F1,T_ref(2,:),t);
T_ref_f(3,:) = lsim(F1,T_ref(3,:),t);



%% REAL SYSTEM Filter
h_f(1,:) = lsim(F1,h(1,:),t);
h_f(2,:) = lsim(F1,h(2,:),t);
h_f(3,:) = lsim(F1,h(3,:),t);

%% REAL SYSTEM Filter
eul_f(1,:) = lsim(F1,eul(1,:),t);
eul_f(2,:) = lsim(F1,eul(2,:),t);
eul_f(3,:) = lsim(F1,eul(3,:),t);


%% REAL SYSTEM_P Filter
eul_p_f(1,:) = lsim(F1,eul_p(1,:),t);
eul_p_f(2,:) = lsim(F1,eul_p(2,:),t);
eul_p_f(3,:) = lsim(F1,eul_p(3,:),t);

v_f(1,:) = lsim(F1,v(1,:),t);
v_f(2,:) = lsim(F1,v(2,:),t);
v_f(3,:) = lsim(F1,v(3,:),t);

v_p_f(1,:) = lsim(F1,v_p(1,:),t);
v_p_f(2,:) = lsim(F1,v_p(2,:),t);
v_p_f(3,:) = lsim(F1,v_p(3,:),t);


%% REAL SYSTEM_PP Filter
eul_pp_f(1,:) = lsim(F1,eul_pp(1,:),t);
eul_pp_f(2,:) = lsim(F1,eul_pp(2,:),t);
eul_pp_f(3,:) = lsim(F1,eul_pp(3,:),t);


%% Parameter matrices
alpha = 0.01;
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
% x0 = ones(1,4);
% %x0 = [ones(1,3),chix, chiy,chiz];
% f_obj1 = @(x) funcion_costo_full(x,u_ref_f,eul_f,v_f,v_p_f,N);                                  
% chi = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
 %%


%%
% chi =  1.0e+03 * [ -0.0937   -0.2444   -0.0654    3.4581   -3.4911   -0.7514    0.6218    0.1065    0.4892    0.2573    1.9080   -2.0800   -0.3292    0.3187   -0.2839 0.0687    0.2530    0.3755   -0.2393   -0.0171   -0.0349    0.0105    0.2049    0.0013]';
chi = [ 0.0002    2.2455    2.1997    1.0160]
x_estimate(:, 1) = [h(:,1);v(:,1)];

for k=1:length(t)-1
      
    x_estimate(:, k+1) =  RK4_euler_vel(x_estimate(:, k), u_ref_f(:, k), chi, ts, eul_f(:,k));
    
end

%%
% Primer subplot: x_estimate y phi_p
subplot(3,1,1)
plot(x_estimate(1,:), 'LineWidth', 2, 'DisplayName', 'hx_{estimate}');
hold on
plot(h(1,:), 'LineWidth', 2, 'DisplayName', 'hx_{real}');
ylabel('Valores');
title('Comparación de phi\_estimate y phi\_p');
legend;
grid on

% Segundo subplot: x_estimate y theta_p
subplot(3,1,2)
plot(x_estimate(2,:), 'LineWidth', 2, 'DisplayName', 'hy_{estimate}');
hold on
plot(h(2,:), 'LineWidth', 2, 'DisplayName', 'hy_{real}');
ylabel('Valores');
title('Comparación de theta\_estimate y theta\_p');
legend;
grid on

% Tercer subplot: x_estimate y psi_p
subplot(3,1,3)
plot(x_estimate(3,:), 'LineWidth', 2, 'DisplayName', 'hz_{estimate}');
hold on
plot(h(3,:), 'LineWidth', 2, 'DisplayName', 'hz_{real}');
xlabel('Tiempo');
ylabel('Valores');
title('Comparación de psi\_estimate y psi\_p');
legend;
grid on


%%
figure(2)
% Primer subplot: x_estimate y phi_p
subplot(3,1,1)
plot(x_estimate(4,:), 'LineWidth', 2, 'DisplayName', 'vx_{estimate}');
hold on
plot(v(1,:), 'LineWidth', 2, 'DisplayName', 'vx_{real}');

ylabel('Valores');
title('Comparación de phi\_estimate y phi\');
legend;
grid on

% Segundo subplot: x_estimate y theta_p
subplot(3,1,2)
plot(x_estimate(5,:), 'LineWidth', 2, 'DisplayName', 'vy_{estimate}');
hold on
plot(v(2,:), 'LineWidth', 2, 'DisplayName', 'vy_{real}');


ylabel('Valores');
title('Comparación de theta\_estimate y theta');
legend;
grid on

% Tercer subplot: x_estimate y psi_p
subplot(3,1,3)
plot(x_estimate(6,:), 'LineWidth', 2, 'DisplayName', 'vz_{estimate}');
hold on
plot(v(3,:), 'LineWidth', 2, 'DisplayName', 'vz_{real}');
hold on
plot(u_ref(3,:), 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1, 'DisplayName', 'vz_{ref}');
xlabel('Tiempo');
ylabel('Valores');
title('Comparación de psi\_estimate y psi\_p');
legend;
grid on