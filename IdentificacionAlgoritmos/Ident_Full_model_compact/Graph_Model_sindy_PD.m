%% IDENTIFICATION OF PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
n = 7;
num = int2str(n);
Archivo_1 = '/home/bryansgue/Doctoral_Research/Python/PySindy_fullcompact/Data_Ident_';
extension = '.mat';

% Cargar los datos del archivo 1
load(strcat(Archivo_1, num, extension));

%%
% load('states_1.mat')
% load('T_ref_1.mat')
% load('t_1.mat')

%%
pose = n;
euler = euler;
vel = v;
euler_p = euler_p;
omega = w;
quat = q;
u = nu;

%% Settings
clear tf;


init = 1;
t = time(1,1:end);

dim = length(t);

t = t(1,init:end);
N = length(t);

ts = 1/30
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
psi = Angulo(double(euler(3,init:dim)));

phi_max = max(abs(phi));
theta_max = max(abs(theta));
psi_max = max(abs(psi));

eul = [phi;theta; psi];
%%
q = [hx;hy;hz;phi;theta; psi];

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
%%
q_p = [vx;vy;vz;phi_p;theta_p; psi_p];

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

q_pp = [vx_p;vy_p;vz_p;phi_pp;theta_pp; psi_pp];
%% Filter signals
landa = 5;  %identi con 25
F1=tf(landa,[1 landa]);

%% REFERENCE SIGNALS Filtradas
u_ref_f(1,:) = lsim(F1,u_ref(1,:),t);
u_ref_f(2,:) = lsim(F1,u_ref(2,:),t);
u_ref_f(3,:) = lsim(F1,u_ref(3,:),t);


T_ref_f(1,:) = lsim(F1,T_ref(1,:),t);
T_ref_f(2,:) = lsim(F1,T_ref(2,:),t);
T_ref_f(3,:) = lsim(F1,T_ref(3,:),t);


input = [u_ref_f;T_ref_f];
input2 = [u_ref;T_ref];

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


%% REFERENCE SIGNALS Filtradas


%% REAL SYSTEM Filter
q_f(1,:) = lsim(F1,q(1,:),t);
q_f(2,:) = lsim(F1,q(2,:),t);
q_f(3,:) = lsim(F1,q(3,:),t);
q_f(4,:) = lsim(F1,q(4,:),t);
q_f(5,:) = lsim(F1,q(5,:),t);
q_f(6,:) = lsim(F1,q(6,:),t);

%% REAL SYSTEM_P Filter
q_p_f(1,:) = lsim(F1,q_p(1,:),t);
q_p_f(2,:) = lsim(F1,q_p(2,:),t);
q_p_f(3,:) = lsim(F1,q_p(3,:),t);
q_p_f(4,:) = lsim(F1,q_p(4,:),t);
q_p_f(5,:) = lsim(F1,q_p(5,:),t);
q_p_f(6,:) = lsim(F1,q_p(6,:),t);

%% REAL SYSTEM_PP Filter
q_pp_f(1,:) = lsim(F1,q_pp(1,:),t);
q_pp_f(2,:) = lsim(F1,q_pp(2,:),t);
q_pp_f(3,:) = lsim(F1,q_pp(3,:),t);
q_pp_f(4,:) = lsim(F1,q_pp(4,:),t);
q_pp_f(5,:) = lsim(F1,q_pp(5,:),t);
q_pp_f(6,:) = lsim(F1,q_pp(6,:),t);

%% Optimizador
load('chi_uav_compact_full_model.mat');

%%
%% ACCIONES DE CONTROL TEST 

% SIMULATION DYNAMICS
q_estimate(:,1) = [q(:,1); q_p(:,1)];   
for k=1:length(t)
    q_estimate(:, k+1) =  f_dynamic(q_estimate(:, k), input(:, k), values_final, ts);
    %q_estimate(6, k+1) = Angulo(q_estimate(6, k+1)); 
    
    % Derivar las posiciones para obtener velocidades y derivar las velocidades para obtener aceleraciones
    if k > 1
        q_estimate_derivado(:, k) = (q_estimate(:, k+1) - q_estimate(:, k)) / ts;
    end
end


%coe = readmatrix('/home/bryansgue/Doctoral_Research/Python/MPC_Syndi_model_UAV/coe_matrix.csv');
x_dot_test_predicted = readmatrix('/home/bryansgue/Doctoral_Research/Python/PySindy_fullcompact/x_dot_test_predicted.csv');
x_dot_test_predicted = x_dot_test_predicted';

x_test_sim = readmatrix('/home/bryansgue/Doctoral_Research/Python/PySindy_fullcompact/x_test_sim.csv');
x_test_sim = x_test_sim';

nx_sindy = x_test_sim(1,:);
ny_sindy = x_test_sim(2,:);
nz_sindy = x_test_sim(3,:);
phi_sindy = x_test_sim(4,:);
theta_sindy = x_test_sim(5,:);
psi_sindy = x_test_sim(6,:);

nx_p_sindy = x_dot_test_predicted(1,:);
ny_p_sindy = x_dot_test_predicted(2,:);
nz_p_sindy = x_dot_test_predicted(3,:);
phi_p_sindy = x_dot_test_predicted(4,:);
theta_p_sindy = x_dot_test_predicted(5,:);
psi_p_sindy = x_dot_test_predicted(6,:);

nx_pp_sindy = x_dot_test_predicted(7,:);
ny_pp_sindy = x_dot_test_predicted(8,:);
nz_pp_sindy = x_dot_test_predicted(9,:);
phi_pp_sindy = x_dot_test_predicted(10,:);
theta_pp_sindy = x_dot_test_predicted(11,:);
psi_pp_sindy = x_dot_test_predicted(12,:);


%%

% Parameters fancy plots
% Figure properties
lw = 1.5; % Line width
fontsizeLabel = 11; 
fontsizeLegend = 11;
fontsizeTicks = 11;
fontsizeTitle = 14;
sizeX = 45; % Ancho en cm para IEEE de doble columna
sizeY = 15;   % Alto en cm

% Convertir de cm a pulgadas para MATLAB
sizeX_inch = sizeX / 2.54;
sizeY_inch = sizeY / 2.54;




% color propreties
C1 = [246 170 141]/255;
C2 = [51 187 238]/255;
C3 = [0 153 136]/255;
C4 = [238 119 51]/255;
C5 = [204 51 17]/255;
C6 = [238 51 119]/255;
C7 = [187 187 187]/255;
C8 = [80 80 80]/255;
C9 = [140 140 140]/255;
C10 = [0 128 255]/255;
C11 = [234 52 89]/255;
C12 = [39 124 252]/255;
C13 = [40 122 125]/255;
%C14 = [86 215 219]/255;
C14 = [252 94 158]/255;
C15 = [244 171 39]/255;
C16 = [100 121 162]/255;
C17 = [255 0 0]/255;

gray_color = [128, 128, 128] / 255;

%% GRAFICA 1
% Crear una figura con 3 filas y 2 columnas ajustando el espaciado
figure('Units', 'inches', 'Position', [1, 1, sizeX_inch, sizeY_inch], 'Color', 'w');

% Ajustar la distancia entre columnas
tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

% Primera subgráfica en la primera columna, primera fila
nexttile
plot(t,hx,'-','Color',C5,'LineWidth',lw); hold on;
plot(t,q_estimate(1,1:length(t)),'--','Color',C3,'LineWidth',lw);
plot(t(1:length(nx_sindy(1,:))),nx_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
ylabel('$[m]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$\textrm{Position }\mathbf{\eta}$'},'fontsize',fontsizeTitle,'interpreter','latex')
legend({'$\eta_x$','$\eta_{x_{PD}}$', '$\eta_{x_{sindy}}$'},'interpreter','latex','fontsize',fontsizeLegend)
grid minor;

% Subgráfica de posición angular en phi (izquierda)
nexttile
plot(t,input(4,:),'--','Color',gray_color,'LineWidth',1); hold on;
plot(t,phi(1,:),'-','Color',C5,'LineWidth',lw); hold on;
plot(t,q_estimate(4,1:length(t)),'--','Color',C3,'LineWidth',lw); hold on;
plot(t(1:length(phi_sindy(1,:))),phi_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
grid minor;
ylabel('$[rad]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$$\textrm{Orientation }\mathbf{\Omega}$'},'fontsize',fontsizeTitle,'interpreter','latex')
legend({'$\phi_{ref}$','$\phi$','$\phi_{PD}$','$\phi_{sindy}$'},'interpreter','latex','fontsize',fontsizeLegend)


% Segunda subgráfica en la primera columna, segunda fila
nexttile
plot(t,hy,'-','Color',C5,'LineWidth',lw); hold on;
plot(t,q_estimate(2,1:length(t)),'--','Color',C3,'LineWidth',lw);
plot(t(1:length(ny_sindy(1,:))),ny_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
grid minor;
ylabel('$[m]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(b)'},'fontsize',fontsizeTitle,'interpreter','latex')
legend({'$\eta_y$','$\eta_{y_{PD}}$', '$\eta_{y_{sindy}}$'},'interpreter','latex','fontsize',fontsizeLegend)


% Segunda subgráfica en la segunda columna, segunda fila
% Subgráfica de posición angular en theta (izquierda)
nexttile
plot(t,input(5,:),'--','Color',gray_color,'LineWidth',1); hold on;
plot(t,theta(1,:),'-','Color',C5,'LineWidth',lw); hold on;
plot(t,q_estimate(5,1:length(t)),'--','Color',C3,'LineWidth',lw); hold on;
plot(t(1:length(theta_sindy(1,:))),theta_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
grid minor;
ylabel('$[rad]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(c) $\theta$'},'fontsize',fontsizeTitle,'interpreter','latex')
legend({'$\theta_{ref}$','$\theta$','$\theta_{PD}$','$\theta_{sindy}$'},'interpreter','latex','fontsize',fontsizeLegend)


% Tercera subgráfica en la primera columna, tercera fila
nexttile
plot(t,hz,'-','Color',C5,'LineWidth',lw); hold on;
plot(t,q_estimate(3,1:length(t)),'--','Color',C3,'LineWidth',lw);
plot(t(1:length(nz_sindy(1,:))),nz_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
grid minor;
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(c)'},'fontsize',fontsizeTitle,'interpreter','latex')
legend({'$\eta_z$','$\eta_{z_{PD}}$', '$\eta_{z_{sindy}}$'},'interpreter','latex','fontsize',fontsizeLegend)


% Tercera subgráfica en la segunda columna, tercera fila

% Subgráfica de posición angular en psi (izquierda)
nexttile
plot(t,psi(1,:),'-','Color',C5,'LineWidth',lw); hold on;
plot(t,q_estimate(6,1:length(t)),'--','Color',C3,'LineWidth',lw); hold on;
plot(t(1:length(psi_sindy(1,:))),psi_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
grid minor;
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(e) $\psi$'},'fontsize',fontsizeTitle,'interpreter','latex')
legend({'$\psi_{ref}$','$\psi$','$\psi_{PD}$','$\psi_{sindy}$'},'interpreter','latex','fontsize',fontsizeLegend)

export_fig a_Posiciones_PD_sindy.pdf -q101

%% GRAFICA 2

% Crear una figura con 3 filas y 2 columnas ajustando el espaciado
figure('Units', 'inches', 'Position', [1, 1, sizeX_inch, sizeY_inch], 'Color', 'w');

% Ajustar la distancia entre columnas
tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

% Primera fila: Phi (Posición y Velocidad)
% Primera subgráfica en la segunda columna, primera fila
nexttile
plot(t,vx(1,:),'-','Color',C5,'LineWidth',lw); hold on;
plot(t,q_estimate(7,1:length(t)),'--','Color',C3,'LineWidth',lw); hold on;
plot(t(1:length(nx_p_sindy(1,:))),nx_p_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
grid minor;
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$\textrm{Linear velocity }\dot{\mathbf{\eta}}$'},'fontsize',fontsizeTitle,'interpreter','latex')
legend({'$\dot{\eta}_x$','$\dot{\eta}_{x_{PD}}$','$\dot{\eta}_{x_{sindy}}$'},'interpreter','latex','fontsize',fontsizeLegend)


% Subgráfica de velocidad angular en phi (derecha)
nexttile
plot(t,phi_p(1,:),'-','Color',C5,'LineWidth',lw); hold on;
plot(t,q_estimate(10,1:length(t)),'--','Color',C3,'LineWidth',lw); hold on;
plot(t(1:length(phi_p_sindy(1,:))),phi_p_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
grid minor;
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$\textrm{Angular velocity }\dot{\mathbf{\Omega}}$'},'fontsize',fontsizeTitle,'interpreter','latex')
legend({'$\dot{\phi}$','$\dot{\phi}_{PD}$','$\dot{\phi}_{sindy}$'},'interpreter','latex','fontsize',fontsizeLegend)

% Segunda fila: Theta (Posición y Velocidad)
nexttile
plot(t,vy(1,:),'-','Color',C5,'LineWidth',lw); hold on;
plot(t,q_estimate(8,1:length(t)),'--','Color',C3,'LineWidth',lw);
plot(t(1:length(ny_p_sindy(1,:))),ny_p_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
grid minor;
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(b)'},'fontsize',fontsizeTitle,'interpreter','latex')
legend({'$\dot{\eta}_y$','$\dot{\eta}_{y_{PD}}$','$\dot{\eta}_{y_{sindy}}$'},'interpreter','latex','fontsize',fontsizeLegend)



% Subgráfica de velocidad angular en theta (derecha)
nexttile
plot(t,theta_p(1,:),'-','Color',C5,'LineWidth',lw); hold on;
plot(t,q_estimate(11,1:length(t)),'--','Color',C3,'LineWidth',lw); hold on;
plot(t(1:length(theta_p_sindy(1,:))),theta_p_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
grid minor;
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(d) $\dot{\theta}$'},'fontsize',fontsizeTitle,'interpreter','latex')
legend({'$\dot{\theta}$','$\dot{\theta}_{PD}$','$\dot{\theta}_{sindy}$'},'interpreter','latex','fontsize',fontsizeLegend)

% Tercera fila: Psi (Posición y Velocidad)
nexttile
plot(t,input(3,:),'--','Color',gray_color,'LineWidth',2); hold on;
plot(t,vz(1,:),'-','Color',C5,'LineWidth',lw); hold on;
plot(t,q_estimate(9,1:length(t)),'--','Color',C3,'LineWidth',lw);
plot(t(1:length(nz_p_sindy(1,:))),nz_p_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
grid minor;
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(c)'},'fontsize',fontsizeTitle,'interpreter','latex')
legend({'$\dot{\eta}_{z_{ref}}$','$\dot{\eta}_z$','$\dot{\eta}_{x_{PD}}$','$\dot{\eta}_{x_{sindy}}$'},'interpreter','latex','fontsize',fontsizeLegend)


% Subgráfica de velocidad angular en psi (derecha)
nexttile
plot(t,input(6,:),'--','Color',gray_color,'LineWidth',1); hold on;
plot(t,psi_p(1,:),'-','Color',C5,'LineWidth',lw); hold on;
plot(t,q_estimate(12,1:length(t)),'--','Color',C3,'LineWidth',lw); hold on;
plot(t(1:length(psi_p_sindy(1,:))),psi_p_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
grid minor;
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(f) $\dot{\psi}$'},'fontsize',fontsizeTitle,'interpreter','latex')
legend({'$\dot{\psi}_{ref}$','$\dot{\psi}$','$\dot{\psi}_{PD}$','$\dot{\psi}_{sindy}$'},'interpreter','latex','fontsize',fontsizeLegend)

export_fig a_Velocidades_PD_sindy.pdf -q101

%% GRAFICA 3

% % Crear una figura con 3 filas y 2 columnas ajustando el espaciado
% figure('Units', 'inches', 'Position', [1, 1, sizeX_inch, sizeY_inch], 'Color', 'w');
% 
% % Ajustar la distancia entre columnas
% tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
% 
% % Primera fila: Phi (Posición y Velocidad)
% % Primera subgráfica en la segunda columna, primera fila
% nexttile
% plot(t,q_pp_f(1,:),'-','Color',C5,'LineWidth',lw); hold on;
% plot(t,q_estimate_derivado(7,1:length(t)),'--','Color',C3,'LineWidth',lw); hold on;
% plot(t(1:length(nx_pp_sindy(1,:))),nx_pp_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
% grid minor;
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(a)'},'fontsize',fontsizeTitle,'interpreter','latex')
% legend({'$\ddot{\eta}_x$','$\ddot{\eta}_{x_{PD}}$','$\ddot{\eta}_{x_{sindy}}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% 
% % Subgráfica de velocidad angular en phi (derecha)
% nexttile
% plot(t,q_pp_f(4,:),'-','Color',C5,'LineWidth',lw); hold on;
% plot(t,q_estimate_derivado(10,1:length(t)),'--','Color',C3,'LineWidth',lw); hold on;
% plot(t(1:length(phi_pp_sindy(1,:))),phi_pp_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
% grid minor;
% ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(b) $\dot{\phi}$'},'fontsize',fontsizeTitle,'interpreter','latex')
% legend({'$\ddot{\phi}$','$\ddot{\phi}_{PD}$','$\ddot{\phi}_{sindy}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% % Segunda fila: Theta (Posición y Velocidad)
% nexttile
% plot(t,q_pp_f(2,:),'-','Color',C5,'LineWidth',lw); hold on;
% plot(t,q_estimate_derivado(8,1:length(t)),'--','Color',C3,'LineWidth',lw);
% plot(t(1:length(ny_pp_sindy(1,:))),ny_pp_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
% grid minor;
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(b)'},'fontsize',fontsizeTitle,'interpreter','latex')
% legend({'$\ddot{\eta}_y$','$\ddot{\eta}_{y_{PD}}$','$\ddot{\eta}_{y_{sindy}}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% 
% 
% % Subgráfica de velocidad angular en theta (derecha)
% nexttile
% plot(t,q_pp_f(5,:),'-','Color',C5,'LineWidth',lw); hold on;
% plot(t,q_estimate_derivado(11,1:length(t)),'--','Color',C3,'LineWidth',lw); hold on;
% plot(t(1:length(theta_pp_sindy(1,:))),theta_pp_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
% grid minor;
% ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(d) $\ddot{\theta}$'},'fontsize',fontsizeTitle,'interpreter','latex')
% legend({'$\ddot{\theta}$','$\ddot{\theta}_{PD}$','$\ddot{\theta}_{sindy}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% % Tercera fila: Psi (Posición y Velocidad)
% nexttile
% plot(t,q_pp_f(3,:),'--','Color',C5,'LineWidth',2); hold on;
% plot(t,q_estimate_derivado(9,1:length(t)),'--','Color',C3,'LineWidth',lw);
% plot(t(1:length(nz_pp_sindy(1,:))),nz_pp_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
% grid minor;
% xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(c)'},'fontsize',fontsizeTitle,'interpreter','latex')
% legend({'$\ddot{\eta}_{z_{ref}}$','$\ddot{\eta}_z$','$\ddot{\eta}_{x_{PD}}$','$\dot{\eta}_{x_{sindy}}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% 
% 
% % Subgráfica de velocidad angular en psi (derecha)
% nexttile
% plot(t,q_pp_f(6,:),'-','Color',C5,'LineWidth',lw); hold on;
% plot(t,q_estimate_derivado(12,1:length(t)),'--','Color',C3,'LineWidth',lw); hold on;
% plot(t(1:length(psi_pp_sindy(1,:))),psi_pp_sindy(1,:),'--','Color',C12,'LineWidth',1*lw); hold on;
% grid minor;
% xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(f) $\dot{\psi}$'},'fontsize',fontsizeTitle,'interpreter','latex')
% legend({'$\ddot{\psi}_{ref}$','$\ddot{\psi}$','$\ddot{\psi}_{PD}$','$\ddot{\psi}_{sindy}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% 
% export_fig a_Aceleraciones_PD_sindy.pdf -q101

%%
%% Truncar las muestras adicionales de las velocidades predichas por SINDy
nx_p_sindy = nx_p_sindy(1:end-1);  % De 1800 a 1799 muestras
ny_p_sindy = ny_p_sindy(1:end-1);  % De 1800 a 1799 muestras
nz_p_sindy = nz_p_sindy(1:end-1);  % De 1800 a 1799 muestras
phi_p_sindy = phi_p_sindy(1:end-1);  % De 1800 a 1799 muestras
theta_p_sindy = theta_p_sindy(1:end-1);  % De 1800 a 1799 muestras
psi_p_sindy = psi_p_sindy(1:end-1);  % De 1800 a 1799 muestras

% Datos reales (obtenidos de la simulación o mediciones)
y_real = [hx; hy; hz; vx; vy; vz; phi; theta; psi; phi_p; theta_p; psi_p];
y_real_r = y_real(:,1:end-2);  % Ajustar las dimensiones a 1799 muestras

% Datos predichos por el modelo SINDy (estimaciones)
y_pred_r = [nx_sindy; ny_sindy; nz_sindy; nx_p_sindy; ny_p_sindy; nz_p_sindy; ...
          phi_sindy; theta_sindy; psi_sindy; phi_p_sindy; theta_p_sindy; psi_p_sindy];

% Datos predichos por el controlador PD
q_estimate = q_estimate(1:12, 1:end);  % Los estados predichos por el PD
q_estimate_r = q_estimate(:,1:end-3);  % Ajustar las dimensiones

%%

% Calcular la media de los valores reales para cada estado
mean_real = mean(y_real_r, 2);

% Calcular RMSE para cada estado (Real vs SINDy y Real vs PD)
RMSE_SINDy = sqrt(mean((y_real_r - y_pred_r).^2, 2));      % RMSE entre real y SINDy
RMSE_PD = sqrt(mean((y_real_r - q_estimate_r).^2, 2));     % RMSE entre real y PD

% Calcular MAE para cada estado (Real vs SINDy y Real vs PD)
MAE_SINDy = mean(abs(y_real_r - y_pred_r), 2);      % MAE entre real y SINDy
MAE_PD = mean(abs(y_real_r - q_estimate_r), 2);     % MAE entre real y PD

% Índice de Concordancia (d)
d_SINDy = 1 - sum(abs(y_pred_r - y_real_r), 2) ./ (sum(abs(y_pred_r - mean_real) + abs(y_real_r - mean_real), 2));
d_PD = 1 - sum(abs(q_estimate_r - y_real_r), 2) ./ (sum(abs(q_estimate_r - mean_real) + abs(y_real_r - mean_real), 2));


% Calcular RMSE global (Root Mean Squared Error)
RMSE_Global_SINDy = sqrt(mean(RMSE_SINDy.^2));
RMSE_Global_PD = sqrt(mean(RMSE_PD.^2));

% Calcular MAE global (Mean Absolute Error)
MAE_Global_SINDy = mean(MAE_SINDy);
MAE_Global_PD = mean(MAE_PD);

% Índice de Concordancia global
d_Global_SINDy = mean(d_SINDy);
d_Global_PD = mean(d_PD);


% Mostrar los resultados
fprintf('RMSE (Real vs SINDy): \n');
disp(RMSE_SINDy);
fprintf('RMSE (Real vs PD): \n');
disp(RMSE_PD);

fprintf('MAE (Real vs SINDy): \n');
disp(MAE_SINDy);
fprintf('MAE (Real vs PD): \n');
disp(MAE_PD);

% Mostrar los resultados
fprintf('Índice de Concordancia (d) - Real vs SINDy: \n');
disp(d_SINDy);
fprintf('Índice de Concordancia (d) - Real vs PD: \n');
disp(d_PD);

% Mostrar los resultados globales
fprintf('RMSE Global (SINDy): %.4f\n', RMSE_Global_SINDy);
fprintf('RMSE Global (PD): %.4f\n', RMSE_Global_PD);

fprintf('MAE Global (SINDy): %.4f\n', MAE_Global_SINDy);
fprintf('MAE Global (PD): %.4f\n', MAE_Global_PD);

fprintf('Índice de Concordancia Global (SINDy): %.4f\n', d_Global_SINDy);
fprintf('Índice de Concordancia Global (PD): %.4f\n', d_Global_PD);

% %% Índices de las categorías
% % Índices para posiciones lineales
% pos_lin_idx = [1, 2, 3];  % \eta_x, \eta_y, \eta_z
% 
% % Índices para posiciones angulares (ángulos de Euler)
% pos_ang_idx = [7, 8, 9];  % \phi, \theta, \psi
% 
% % Índices para velocidades lineales
% vel_lin_idx = [4, 5, 6];  % \dot{\eta}_x, \dot{\eta}_y, \dot{\eta}_z
% 
% % Índices para velocidades angulares
% vel_ang_idx = [10, 11, 12];  % \dot{\phi}, \dot{\theta}, \dot{\psi}
% 
% %% Graficar las métricas de error por categorías
% %% Crear una figura para posiciones lineales (RMSE, MAE, Índice de Concordancia (d))
% figure;
% subplot(3, 1, 1);
% bar([RMSE_SINDy(pos_lin_idx) RMSE_PD(pos_lin_idx)]);  % Gráfico de RMSE para posiciones lineales
% set(gca, 'XTickLabel', {'\eta_x', '\eta_y', '\eta_z'});  % Etiquetas para las posiciones lineales
% xlabel('Posiciones Lineales', 'FontSize', 12);
% ylabel('RMSE', 'FontSize', 12);
% title('RMSE - Posiciones Lineales', 'FontSize', 14);
% legend('SINDy', 'PD');
% grid on;
% 
% subplot(3, 1, 2);
% bar([MAE_SINDy(pos_lin_idx) MAE_PD(pos_lin_idx)]);  % Gráfico de MAE para posiciones lineales
% set(gca, 'XTickLabel', {'\eta_x', '\eta_y', '\eta_z'});
% xlabel('Posiciones Lineales', 'FontSize', 12);
% ylabel('MAE', 'FontSize', 12);
% title('MAE - Posiciones Lineales', 'FontSize', 14);
% legend('SINDy', 'PD');
% grid on;
% 
% subplot(3, 1, 3);
% bar([d_SINDy(pos_lin_idx) d_PD(pos_lin_idx)]);  % Gráfico del Índice de Concordancia (d) para posiciones lineales
% set(gca, 'XTickLabel', {'\eta_x', '\eta_y', '\eta_z'});
% xlabel('Posiciones Lineales', 'FontSize', 12);
% ylabel('Índice de Concordancia (d)', 'FontSize', 12);
% title('Índice de Concordancia (d) - Posiciones Lineales', 'FontSize', 14);
% legend('SINDy', 'PD');
% grid on;
% 
% %% Crear una figura para posiciones angulares (RMSE, MAE, Índice de Concordancia (d))
% figure;
% subplot(3, 1, 1);
% bar([RMSE_SINDy(pos_ang_idx) RMSE_PD(pos_ang_idx)]);  % Gráfico de RMSE para posiciones angulares
% set(gca, 'XTickLabel', {'\phi', '\theta', '\psi'});  % Etiquetas para las posiciones angulares
% xlabel('Posiciones Angulares', 'FontSize', 12);
% ylabel('RMSE', 'FontSize', 12);
% title('RMSE - Posiciones Angulares', 'FontSize', 14);
% legend('SINDy', 'PD');
% grid on;
% 
% subplot(3, 1, 2);
% bar([MAE_SINDy(pos_ang_idx) MAE_PD(pos_ang_idx)]);  % Gráfico de MAE para posiciones angulares
% set(gca, 'XTickLabel', {'\phi', '\theta', '\psi'});
% xlabel('Posiciones Angulares', 'FontSize', 12);
% ylabel('MAE', 'FontSize', 12);
% title('MAE - Posiciones Angulares', 'FontSize', 14);
% legend('SINDy', 'PD');
% grid on;
% 
% subplot(3, 1, 3);
% bar([d_SINDy(pos_ang_idx) d_PD(pos_ang_idx)]);  % Gráfico del Índice de Concordancia (d) para posiciones angulares
% set(gca, 'XTickLabel', {'\phi', '\theta', '\psi'});
% xlabel('Posiciones Angulares', 'FontSize', 12);
% ylabel('Índice de Concordancia (d)', 'FontSize', 12);
% title('Índice de Concordancia (d) - Posiciones Angulares', 'FontSize', 14);
% legend('SINDy', 'PD');
% grid on;
% 
% %% Crear una figura para velocidades lineales (RMSE, MAE, Índice de Concordancia (d))
% figure;
% subplot(3, 1, 1);
% bar([RMSE_SINDy(vel_lin_idx) RMSE_PD(vel_lin_idx)]);  % Gráfico de RMSE para velocidades lineales
% set(gca, 'XTickLabel', {'\dot{\eta}_x', '\dot{\eta}_y', '\dot{\eta}_z'});  % Etiquetas para las velocidades lineales
% xlabel('Velocidades Lineales', 'FontSize', 12);
% ylabel('RMSE', 'FontSize', 12);
% title('RMSE - Velocidades Lineales', 'FontSize', 14);
% legend('SINDy', 'PD');
% grid on;
% 
% subplot(3, 1, 2);
% bar([MAE_SINDy(vel_lin_idx) MAE_PD(vel_lin_idx)]);  % Gráfico de MAE para velocidades lineales
% set(gca, 'XTickLabel', {'\dot{\eta}_x', '\dot{\eta}_y', '\dot{\eta}_z'});
% xlabel('Velocidades Lineales', 'FontSize', 12);
% ylabel('MAE', 'FontSize', 12);
% title('MAE - Velocidades Lineales', 'FontSize', 14);
% legend('SINDy', 'PD');
% grid on;
% 
% subplot(3, 1, 3);
% bar([d_SINDy(vel_lin_idx) d_PD(vel_lin_idx)]);  % Gráfico del Índice de Concordancia (d) para velocidades lineales
% set(gca, 'XTickLabel', {'\dot{\eta}_x', '\dot{\eta}_y', '\dot{\eta}_z'});
% xlabel('Velocidades Lineales', 'FontSize', 12);
% ylabel('Índice de Concordancia (d)', 'FontSize', 12);
% title('Índice de Concordancia (d) - Velocidades Lineales', 'FontSize', 14);
% legend('SINDy', 'PD');
% grid on;
% 
% %% Crear una figura para velocidades angulares (RMSE, MAE, Índice de Concordancia (d))
% figure;
% subplot(3, 1, 1);
% bar([RMSE_SINDy(vel_ang_idx) RMSE_PD(vel_ang_idx)]);  % Gráfico de RMSE para velocidades angulares
% set(gca, 'XTickLabel', {'\dot{\phi}', '\dot{\theta}', '\dot{\psi}'});  % Etiquetas para las velocidades angulares
% xlabel('Velocidades Angulares', 'FontSize', 12);
% ylabel('RMSE', 'FontSize', 12);
% title('RMSE - Velocidades Angulares', 'FontSize', 14);
% legend('SINDy', 'PD');
% grid on;
% 
% subplot(3, 1, 2);
% bar([MAE_SINDy(vel_ang_idx) MAE_PD(vel_ang_idx)]);  % Gráfico de MAE para velocidades angulares
% set(gca, 'XTickLabel', {'\dot{\phi}', '\dot{\theta}', '\dot{\psi}'});
% xlabel('Velocidades Angulares', 'FontSize', 12);
% ylabel('MAE', 'FontSize', 12);
% title('MAE - Velocidades Angulares', 'FontSize', 14);
% legend('SINDy', 'PD');
% grid on;
% 
% subplot(3, 1, 3);
% bar([d_SINDy(vel_ang_idx) d_PD(vel_ang_idx)]);  % Gráfico del Índice de Concordancia (d) para velocidades angulares
% set(gca, 'XTickLabel', {'\dot{\phi}', '\dot{\theta}', '\dot{\psi}'});
% xlabel('Velocidades Angulares', 'FontSize', 12);
% ylabel('Índice de Concordancia (d)', 'FontSize', 12);
% title('Índice de Concordancia (d) - Velocidades Angulares', 'FontSize', 14);
% legend('SINDy', 'PD');
% grid on;
% 
% 
% %%
% % Ajustar los parámetros para el tamaño de fuente
% lw = 1.5; % Line width
% fontsizeLabel = 11; 
% fontsizeLegend = 10;
% fontsizeTicks = 11;
% fontsizeTitle = 14;
% sizeX = 45; % Ancho en cm para IEEE de doble columna
% sizeY = 15;   % Alto en cm
% 
% % Permitir que el tamaño de letra dependa de un valor definido por el usuario
% labelFontSize = 14;  % Este valor puede ser modificado por el usuario
% 
% % Convertir de cm a pulgadas para MATLAB
% sizeX_inch = sizeX / 2.54;
% sizeY_inch = sizeY / 2.54;
% 
% % Configurar el layout de la figura
% figure('Units', 'inches', 'Position', [1, 1, sizeX_inch, sizeY_inch], 'Color', 'w');
% t = tiledlayout(3, 4, 'TileSpacing', 'compact', 'Padding', 'compact'); % 3 filas, 4 columnas
% 
% % Título general
% %title(t, 'Comparison of RMSE, MAE, and Concordance Index for SINDy and PD', 'FontSize', fontsizeTitle);
% 
% % --- Primera columna: Posiciones Lineales ---
% % RMSE - Posiciones Lineales
% nexttile(1);
% b1 = bar([RMSE_SINDy(pos_lin_idx) RMSE_PD(pos_lin_idx)], 'BarWidth', 0.6);
% set(gca, 'XTickLabel', {'$\eta_x$', '$\eta_y$', '$\eta_z$'}, 'TickLabelInterpreter', 'latex', 'FontSize', labelFontSize);
% title('Linear Positions', 'Interpreter', 'latex', 'FontSize', labelFontSize);
% ylabel('RMSE', 'Interpreter', 'latex', 'FontSize', labelFontSize);
% grid on; grid minor;
% 
% % MAE - Posiciones Lineales
% nexttile(5);
% b2 = bar([MAE_SINDy(pos_lin_idx) MAE_PD(pos_lin_idx)], 'BarWidth', 0.6);
% set(gca, 'XTickLabel', {'$\eta_x$', '$\eta_y$', '$\eta_z$'}, 'TickLabelInterpreter', 'latex', 'FontSize', labelFontSize);
% ylabel('MAE', 'Interpreter', 'latex', 'FontSize', labelFontSize);
% grid on; grid minor;
% 
% % d - Posiciones Lineales
% nexttile(9);
% b3 = bar([d_SINDy(pos_lin_idx) d_PD(pos_lin_idx)], 'BarWidth', 0.6);
% set(gca, 'XTickLabel', {'$\eta_x$', '$\eta_y$', '$\eta_z$'}, 'TickLabelInterpreter', 'latex', 'FontSize', labelFontSize);
% ylabel('Concordance Index', 'Interpreter', 'latex', 'FontSize', labelFontSize);
% grid on; grid minor;
% 
% % --- Segunda columna: Posiciones Angulares ---
% % RMSE - Posiciones Angulares
% nexttile(2);
% bar([RMSE_SINDy(pos_ang_idx) RMSE_PD(pos_ang_idx)], 'BarWidth', 0.6);
% set(gca, 'XTickLabel', {'$\phi$', '$\theta$', '$\psi$'}, 'TickLabelInterpreter', 'latex', 'FontSize', labelFontSize);
% title('Angular Positions', 'Interpreter', 'latex', 'FontSize', labelFontSize);
% grid on; grid minor;
% 
% % MAE - Posiciones Angulares
% nexttile(6);
% bar([MAE_SINDy(pos_ang_idx) MAE_PD(pos_ang_idx)], 'BarWidth', 0.6);
% set(gca, 'XTickLabel', {'$\phi$', '$\theta$', '$\psi$'}, 'TickLabelInterpreter', 'latex', 'FontSize', labelFontSize);
% grid on; grid minor;
% 
% % d - Posiciones Angulares
% nexttile(10);
% bar([d_SINDy(pos_ang_idx) d_PD(pos_ang_idx)], 'BarWidth', 0.6);
% set(gca, 'XTickLabel', {'$\phi$', '$\theta$', '$\psi$'}, 'TickLabelInterpreter', 'latex', 'FontSize', labelFontSize);
% grid on; grid minor;
% 
% % --- Tercera columna: Velocidades Lineales ---
% % RMSE - Velocidades Lineales
% nexttile(3);
% bar([RMSE_SINDy(vel_lin_idx) RMSE_PD(vel_lin_idx)], 'BarWidth', 0.6);
% set(gca, 'XTickLabel', {'$\dot{\eta}_x$', '$\dot{\eta}_y$', '$\dot{\eta}_z$'}, 'TickLabelInterpreter', 'latex', 'FontSize', labelFontSize);
% title('Linear Velocities', 'Interpreter', 'latex', 'FontSize', labelFontSize);
% grid on; grid minor;
% 
% % MAE - Velocidades Lineales
% nexttile(7);
% bar([MAE_SINDy(vel_lin_idx) MAE_PD(vel_lin_idx)], 'BarWidth', 0.6);
% set(gca, 'XTickLabel', {'$\dot{\eta}_x$', '$\dot{\eta}_y$', '$\dot{\eta}_z$'}, 'TickLabelInterpreter', 'latex', 'FontSize', labelFontSize);
% grid on; grid minor;
% 
% % d - Velocidades Lineales
% nexttile(11);
% bar([d_SINDy(vel_lin_idx) d_PD(vel_lin_idx)], 'BarWidth', 0.6);
% set(gca, 'XTickLabel', {'$\dot{\eta}_x$', '$\dot{\eta}_y$', '$\dot{\eta}_z$'}, 'TickLabelInterpreter', 'latex', 'FontSize', labelFontSize);
% grid on; grid minor;
% 
% % --- Cuarta columna: Velocidades Angulares ---
% % RMSE - Velocidades Angulares
% nexttile(4);
% bar([RMSE_SINDy(vel_ang_idx) RMSE_PD(vel_ang_idx)], 'BarWidth', 0.6);
% set(gca, 'XTickLabel', {'$\dot{\phi}$', '$\dot{\theta}$', '$\dot{\psi}$'}, 'TickLabelInterpreter', 'latex', 'FontSize', labelFontSize);
% title('Angular Velocities', 'Interpreter', 'latex', 'FontSize', labelFontSize);
% grid on; grid minor;
% 
% % MAE - Velocidades Angulares
% nexttile(8);
% bar([MAE_SINDy(vel_ang_idx) MAE_PD(vel_ang_idx)], 'BarWidth', 0.6);
% set(gca, 'XTickLabel', {'$\dot{\phi}$', '$\dot{\theta}$', '$\dot{\psi}$'}, 'TickLabelInterpreter', 'latex', 'FontSize', labelFontSize);
% grid on; grid minor;
% 
% % d - Velocidades Angulares
% nexttile(12);
% bar([d_SINDy(vel_ang_idx) d_PD(vel_ang_idx)], 'BarWidth', 0.6);
% set(gca, 'XTickLabel', {'$\dot{\phi}$', '$\dot{\theta}$', '$\dot{\psi}$'}, 'TickLabelInterpreter', 'latex', 'FontSize', labelFontSize);
% grid on; grid minor;
% 
% % Añadir leyenda global
% legend(t.Children(end), {'SINDy', 'PD'}, 'Orientation', 'horizontal', 'Location', 'southoutside', 'Interpreter', 'latex', 'FontSize', fontsizeLegend);
% 
