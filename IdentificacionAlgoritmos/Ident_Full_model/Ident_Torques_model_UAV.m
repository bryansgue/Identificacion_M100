
%% IDENTIFICATION OF PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
n=4;
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
h = states(1:3,:);
euler = states(4:6,:);
v = states(7:9,:);
euler_p = states(10:12,:);
omega = states(13:15,:);
quat = states(16:19,:);
u = states(20:22,:);

%% Settings
clear tf;

final = 0;
init = 5;
t = t(1,1:end-final);

dim = length(t);

t = t(1,init:end);
N = length(t);

ts = 0.03;

%% REFERENCE SIGNALS 

phi_ref = T_ref(2,init:dim);
theta_ref = T_ref(3,init:dim);
psi_p_ref = T_ref(4,init:dim);

phi_ref_max = max(abs(phi_ref));
theta_ref_max = max(abs(theta_ref));
psi_p_ref_max = max(abs(psi_p_ref));

% REFERENCE SIGNALS
T_ref = [phi_ref; theta_ref; psi_p_ref];

%% REAL SYSTEM states

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

%% REAL SYSTEM ACCELERATIONS
phi_pp = [0, diff(phi_p)/ts];
theta_pp = [0 , diff(theta_p)/ts];
psi_pp = [0 , diff(psi_p)/ts];

phi_pp_max = max(abs(phi_pp));
theta_pp_max = max(abs(theta_pp));
psi_pp_max = max(abs(psi_pp));

eul_pp =  [phi_pp;theta_pp; psi_pp];

%% Filter signals
landa = 25;
F1=tf(landa,[1 landa]);

%% REFERENCE SIGNALS Filtradas

u_ref_f(1,:) = lsim(F1,T_ref(1,:),t);
u_ref_f(2,:) = lsim(F1,T_ref(2,:),t);
u_ref_f(3,:) = lsim(F1,T_ref(3,:),t);


%% REAL SYSTEM Filter
eul_f(1,:) = lsim(F1,eul(1,:),t);
eul_f(2,:) = lsim(F1,eul(2,:),t);
eul_f(3,:) = lsim(F1,eul(3,:),t);


%% REAL SYSTEM_P Filter
eul_p_f(1,:) = lsim(F1,eul_p(1,:),t);
eul_p_f(2,:) = lsim(F1,eul_p(2,:),t);
eul_p_f(3,:) = lsim(F1,eul_p(3,:),t);


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
%     'MaxFunEvals', 5000,... % Agregado nuevo_valor aquí
%     'Algorithm', 'active-set',...
%     'FinDiffType', 'forward',...
%     'RelLineSrchBnd', [],...
%     'RelLineSrchBndDuration', 1,...
%     'TolConSQP', 2e-8);
% 
% x0 = [2.4393    2.1479    2.3460   78.4520 -105.0229  -16.5443   -7.3488    3.1606    2.4795    9.5133    1.4298   -1.6360   -0.2525   -0.1226 2.2615    0.0044    0.2542    1.8579   -1.1822    0.2599   -0.7527    0.3357   -1.6847    0.0179].*rand(1,24);
% 
% % x0 = ones(1,24).*rand(1,24);
% %x0 = [ones(1,3),chix, chiy,chiz];
% f_obj1 = @(x) funcion_costo(x, u_ref_f, eul_f, eul_p_f,eul_pp_f, N);                                  
% chi = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
 %%


%%
 chi = [-0.1140   -4.6592   -2.2340   62.2132  -62.3365  -14.2061   -7.0005    2.8220    3.1867    2.3961   15.9205  -18.1003   -2.9719   -4.1456 -2.0703   -0.4450    2.9815   31.6103  -20.0883   21.0181  -35.1003   -0.3361   13.4542    1.8608];

x_estimate(:, 1) = [eul(:,1);eul_p(:,1)];

for k=1:length(t)-1
      
    x_estimate(:, k+1) =  RK4_euler_uav(x_estimate(:, k), u_ref_f(:, k), chi, ts) ;
    
end

%%
% Primer subplot: x_estimate y phi_p
subplot(3,1,1)
plot(x_estimate(4,:), 'LineWidth', 2, 'DisplayName', 'phi\_pestimate');
hold on
plot(phi_p(1,:), 'LineWidth', 2, 'DisplayName', 'phi\_p');
ylabel('Valores');
title('Comparación de phi\_estimate y phi\_p');
legend;
grid on

% Segundo subplot: x_estimate y theta_p
subplot(3,1,2)
plot(x_estimate(5,:), 'LineWidth', 2, 'DisplayName', 'theta\_pestimate');
hold on
plot(theta_p(1,:), 'LineWidth', 2, 'DisplayName', 'theta\_p');
ylabel('Valores');
title('Comparación de theta\_estimate y theta\_p');
legend;
grid on

% Tercer subplot: x_estimate y psi_p
subplot(3,1,3)
plot(x_estimate(6,:), 'LineWidth', 2, 'DisplayName', 'psi\_pestimate');
hold on
plot(psi_p(1,:), 'LineWidth', 2, 'DisplayName', 'psi\_p');
xlabel('Tiempo');
ylabel('Valores');
title('Comparación de psi\_estimate y psi\_p');
legend;
grid on


%%
figure(2)
% Primer subplot: x_estimate y phi_p
subplot(3,1,1)
plot(x_estimate(1,:), 'LineWidth', 2, 'DisplayName', 'phi\estimate');
hold on
plot(phi(1,:), 'LineWidth', 2, 'DisplayName', 'phi');
hold on
plot(phi_ref(1,:), 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1, 'DisplayName', 'phi');
ylabel('Valores');
title('Comparación de phi\_estimate y phi\');
legend;
grid on

% Segundo subplot: x_estimate y theta_p
subplot(3,1,2)
plot(x_estimate(2,:), 'LineWidth', 2, 'DisplayName', 'theta\estimate');
hold on
plot(theta(1,:), 'LineWidth', 2, 'DisplayName', 'theta');
hold on
plot(theta_ref(1,:),'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1, 'DisplayName', 'phi');
ylabel('Valores');
title('Comparación de theta\_estimate y theta');
legend;
grid on

% Tercer subplot: x_estimate y psi_p
subplot(3,1,3)
plot(x_estimate(3,:), 'LineWidth', 2, 'DisplayName', 'psi\_pestimate');
hold on
plot(Angulo(psi(1,:)), 'LineWidth', 2, 'DisplayName', 'psi');
xlabel('Tiempo');
ylabel('Valores');
title('Comparación de psi\_estimate y psi\_p');
legend;
grid on