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
landa = 25;
F1=tf(landa,[1 landa]);

%% REFERENCE SIGNALS Filtradas
u_ref_f(1,:) = lsim(F1,u_ref(1,:),t);
u_ref_f(2,:) = lsim(F1,u_ref(2,:),t);
u_ref_f(3,:) = lsim(F1,u_ref(3,:),t);


T_ref_f(1,:) = lsim(F1,T_ref(1,:),t);
T_ref_f(2,:) = lsim(F1,T_ref(2,:),t);
T_ref_f(3,:) = lsim(F1,T_ref(3,:),t);


input = [u_ref;T_ref];

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


%%
% OPTIMIZATION PARAMETERS IDENTIFICATION
options = optimoptions(@fmincon,'Algorithm','interior-point'); 
options.MaxFunctionEvaluations = 60000;   
rng default;
ms = MultiStart('FunctionTolerance',2e-4,'UseParallel',true,'Display','iter');
gs = GlobalSearch(ms);

% INITIAL VALUES
% values= load("Xini.mat"); 
chi = ones(16,1);  
f_obj1 = @(x)  funcion_costo_fullUAV(x, input, q_f, q_p_f, q_pp_f, N);
% problem = fmincon(f_obj1,chi,[],[],[],[],[],[],[],options);
problem = createOptimProblem('fmincon','objective',...
    f_obj1,'x0',chi,'lb',[zeros(16,1)],'ub',[],'options',options);
[x, f] = run(gs, problem);
values_final = x;


%% ACCIONES DE CONTROL TEST 
% F_ref =     0.0*ones(1, length(t));
% phi_ref =   0*ones(1, length(t));
% theta_ref = 0*ones(1, length(t));
% psi_ref =   0*ones(1, length(t));
% u_ref = [0*F_ref;0*F_ref;F_ref; phi_ref; theta_ref; psi_ref];

% SIMULATION DYNAMICS
q_estimate(:,1) = [q(:,1); q_p(:,1)];
for k=1:length(t)
    q_estimate(:, k+1) =  f_dynamic(q_estimate(:, k), input(:, k), values_final, ts);
end

% figure
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperPosition', [0 0 8 3]);
% luz = light;
% luz.Color=[0.65,0.65,0.65];
% luz.Style = 'infinite';
% 
% Drone_Parameters(0.02);
% G2=Drone_Plot_3D(q_estimate(1,1),q_estimate(2,1),q_estimate(3,1),q_estimate(4,1), q_estimate(5,1), q_estimate(6,1));hold on
% close all
% 
% axis equal
% for k = 1:100:length(t)-1
%     drawnow
%     delete(G2);
%    
%     G2=Drone_Plot_3D(q_estimate(1,k),q_estimate(2,k),q_estimate(3,k),q_estimate(4,k), q_estimate(5,k), q_estimate(6,k));hold on
% 
%     plot3(q_estimate(1,1:k),q_estimate(2,1:k),q_estimate(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
%     %plot3(obs(1,:),obs(2,:),obs(3,:),'x','Color',[0,171,217]/255,'linewidth',2);
%     legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
%     legend('boxoff')
%     title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
%     xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
%     
% end

% Parameters fancy plots
% define plot properties
lw = 2; % linewidth 1
lwV = 2; % linewidth 2
fontsizeLabel = 9; %11
fontsizeLegend = 9;
fontsizeTicks = 9;
fontsizeTitel = 9;
sizeX = 900; % size figure
sizeY = 300; % size figure

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
% 
% 
figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(3,1,1)
%plot(t(1:length(F_ref)),F_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,hx,'-','Color',C11,'LineWidth',lw); hold on;
plot(t,q_estimate(1,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$hx_{ref}$','$hx_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(3,1,2)
%plot(t(1:length(phi_ref)),phi_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,hy,'-','Color',C13,'LineWidth',lw); hold on;
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
plot(t,q_estimate(2,1:length(t)),'--','Color',C14,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$hy_{ref}$','$hy_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(3,1,3)
%plot(t(1:length(theta_ref)),theta_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,hz,'-','Color',C2,'LineWidth',lw); hold on;
plot(t,q_estimate(3,1:length(t)),'--','Color',C15,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(c)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$hz_{ref}$','$hz_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)



figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(3,1,1)
%plot(t(1:length(F_ref)),F_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,vx(1,:),'-','Color',C11,'LineWidth',lw); hold on;
plot(t,q_estimate(7,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$xp_{ref}$','$xp_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(3,1,2)
%plot(t(1:length(phi_ref)),phi_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,vy(1,:),'-','Color',C13,'LineWidth',lw); hold on;
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
plot(t,q_estimate(8,1:length(t)),'--','Color',C14,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$yp_{ref}$','$yp_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(3,1,3)
plot(t,input(3,:),'--','Color',gray_color,'LineWidth',1); hold on;
plot(t,vz(1,:),'-','Color',C2,'LineWidth',lw); hold on;
plot(t,q_estimate(9,1:length(t)),'--','Color',C15,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(c)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$zp_{ref}$','$zp_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)


figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(3,1,1)
%plot(t(1:length(F_ref)),F_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,input(4,:),'--','Color',gray_color,'LineWidth',1); hold on;
plot(t,phi(1,:),'-','Color',C11,'LineWidth',lw); hold on;
plot(t,q_estimate(4,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\phi_{real}$','$\phi_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(3,1,2)
%plot(t(1:length(phi_ref)),phi_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,input(5,:),'--','Color',gray_color,'LineWidth',1); hold on;
plot(t,theta(1,:),'-','Color',C13,'LineWidth',lw); hold on;
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
plot(t,q_estimate(5,1:length(t)),'--','Color',C14,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\theta_{real}$','$\theta_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(3,1,3)
%plot(t(1:length(theta_ref)),theta_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,psi(1,:),'-','Color',C2,'LineWidth',lw); hold on;
plot(t,q_estimate(6,1:length(t)),'--','Color',C15,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(c)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\psi_{real}$','$\psi_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)

%%%%%%%%

figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(3,1,1)
%plot(t(1:length(F_ref)),F_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,phi_p(1,:),'-','Color',C11,'LineWidth',lw); hold on;
plot(t,q_estimate(10,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\phi_{preal}$','$\phi_{pm}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(3,1,2)
%plot(t(1:length(phi_ref)),phi_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,theta_p(1,:),'-','Color',C13,'LineWidth',lw); hold on;
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
plot(t,q_estimate(11,1:length(t)),'--','Color',C14,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\theta_{preal}$','$\theta_{pm}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(3,1,3)
%plot(t(1:length(theta_ref)),theta_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,input(6,:),'--','Color',gray_color,'LineWidth',1); hold on;
plot(t,psi_p(1,:),'-','Color',C2,'LineWidth',lw); hold on;
plot(t,q_estimate(12,1:length(t)),'--','Color',C15,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(c)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\psi_{preal}$','$\psi_{pm}$'},'interpreter','latex','fontsize',fontsizeLegend)