
%% IDENTIFICATION OF PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
n=3;
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
init = 1;
t = t(1,1:end-final);

dim = length(t);

t = t(1,init:end);
N = length(t);

ts = 0.03;

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
% 
% ul = double(u(1,:));
% um = double(u(2,:));
% un = double(u(3,:));
% % 
% u =  [ul; um; un];

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
landa = 10;
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
%% Parametros del optimizador
options = optimset('Display','iter',...
    'TolFun', 1e-8,...
    'MaxIter', 60000,...
    'MaxFunEvals', 10000,... % Agregado nuevo_valor aquí
    'Algorithm', 'active-set',...
    'FinDiffType', 'forward',...
    'RelLineSrchBnd', [],...
    'RelLineSrchBndDuration', 1,...
    'TolConSQP', 2e-8);

x0 = ones(1,19);
%x0 = [ones(1,3),chix, chiy,chiz];
f_obj1 = @(x) funcion_costo(x, u_ref_f, u_p_f, u_f, omega_f,omega_p_f, N, L); 
                            
chi = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
 %
 
 
 


%% SIMULATION DYNAMICS
x_estimate(:,1) = x(:,1);
for k=1:length(t)
    x_estimate(:, k+1) = dynamic_model_for_sim(chi, x_estimate(:,k), u_ref(:,k), L, ts);
end

%%
% Primer subplot: x_estimate y phi_p
subplot(4,1,1)
plot(x(1,:), 'LineWidth', 2, 'DisplayName', 'ul_{real}');
hold on
plot(x_estimate(1,:), 'LineWidth', 2, 'DisplayName', 'ul_{estimate}');
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
plot(x_estimate(2,:), 'LineWidth', 2, 'DisplayName', 'um_{estimate}');
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
plot(x_estimate(3,:), 'LineWidth', 2, 'DisplayName', 'ul_{estimate}');
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
plot(x_estimate(4,:), 'LineWidth', 2, 'DisplayName', 'r_{estimate}');
hold on
plot(u_ref(4,:),'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1, 'DisplayName', 'r_{ref}');

xlabel('Tiempo');
ylabel('Valores');
title('Comparación de psi\_estimate y psi\_p');
legend;
grid on


