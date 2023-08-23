%% IDENTIFICATION OF PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
n=12;
text1 = 'M100_ident_euler';
n_chr = int2str(n);
test2 = '.mat';
File = strcat(text1,n_chr,test2)
load(File)
%% Settings
clear tf;
desface = 0;
t = t(1,1:end-desface);
N = length(t);

%% REFERENCE SIGNALS 
F_ref = v_ref(1,1:end-desface);
phi_ref = v_ref(2,1:end-desface);
theta_ref = v_ref(3,1:end-desface);
psi_ref = v_ref(4,1:end-desface);

% REFERENCE SIGNALS
u_ref = [0*F_ref;0*F_ref;F_ref; phi_ref; theta_ref; psi_ref];

%% REAL SYSTEM states
hx = double(x(1,1:length(F_ref)));
hy = double(x(2,1:length(phi_ref)));
hz = double(x(3,1:length(theta_ref)));
roll = double(euler(1,1:length(psi_ref)));
pitch = double(euler(2,1:length(psi_ref)));
yaw = double(euler(3,1:length(psi_ref)));

q = [hx; hy; hz; roll;pitch; yaw];

%% REAL SYSTEM State_p
hx_p = double(x_p(1,1:length(F_ref)));
hy_p = double(x_p(2,1:length(phi_ref)));
hz_p = double(x_p(3,1:length(theta_ref)));
roll_p = double(euler_p(1,1:length(psi_ref)));
pitch_p = double(euler_p(2,1:length(psi_ref)));
yaw_p = double(euler_p(3,1:length(psi_ref)));

q_p =  [hx_p; hy_p; hz_p; roll_p ; pitch_p; yaw_p];

%% REAL SYSTEM ACCELERATIONS
hx_pp = [0, diff(hx_p)/ts];
hy_pp = [0 , diff(hy_p)/ts];
hz_pp = [0 , diff(hz_p)/ts];

roll_pp = [0, diff(roll_p)/ts];
pitch_pp = [0 , diff(pitch_p)/ts];
yaw_pp = [0 , diff(yaw_p)/ts];

q_pp =  [hx_pp; hy_pp; hz_pp; roll_pp ; pitch_pp; yaw_pp];

%% Filter signals
landa = 10;
F1=tf(landa,[1 landa]);

%% REFERENCE SIGNALS Filtradas

u_ref_f(1,:) = lsim(F1,u_ref(1,:),t);
u_ref_f(2,:) = lsim(F1,u_ref(2,:),t);
u_ref_f(3,:) = lsim(F1,u_ref(3,:),t);
u_ref_f(4,:) = lsim(F1,u_ref(4,:),t);
u_ref_f(5,:) = lsim(F1,u_ref(5,:),t);
u_ref_f(6,:) = lsim(F1,u_ref(6,:),t);

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
f_obj1 = @(x)  funcion_costo_fullUAV(x, u_ref_f, q_f, q_p_f, q_pp_f, N);
% problem = fmincon(f_obj1,chi,[],[],[],[],[],[],[],options);
problem = createOptimProblem('fmincon','objective',...
    f_obj1,'x0',chi,'lb',[zeros(16,1)],'ub',[],'options',options);
[x, f] = run(gs, problem);
values_final = x;

save("values_final.mat","values_final")
%% ACCIONES DE CONTROL TEST 
% F_ref =     0.0*ones(1, length(t));
% phi_ref =   0*ones(1, length(t));
% theta_ref = 0*ones(1, length(t));
% psi_ref =   0*ones(1, length(t));
% u_ref = [0*F_ref;0*F_ref;F_ref; phi_ref; theta_ref; psi_ref];

% SIMULATION DYNAMICS
q_estimate(:,1) = [q(:,1); q_p(:,1)];
for k=1:length(t)
    q_estimate(:, k+1) =  f_dynamic(q_estimate(:, k), u_ref(:, k), values_final, ts);
end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 8 3]);
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';

Drone_Parameters(0.02);
G2=Drone_Plot_3D(q_estimate(1,1),q_estimate(2,1),q_estimate(3,1),q_estimate(4,1), q_estimate(5,1), q_estimate(6,1));hold on
close all

axis equal
for k = 1:100:length(t)-1
    drawnow
    delete(G2);
   
    G2=Drone_Plot_3D(q_estimate(1,k),q_estimate(2,k),q_estimate(3,k),q_estimate(4,k), q_estimate(5,k), q_estimate(6,k));hold on

    plot3(q_estimate(1,1:k),q_estimate(2,1:k),q_estimate(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    %plot3(obs(1,:),obs(2,:),obs(3,:),'x','Color',[0,171,217]/255,'linewidth',2);
    legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    
end

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
plot(t,q_p(1,:),'-','Color',C11,'LineWidth',lw); hold on;
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
plot(t,q_p(2,:),'-','Color',C13,'LineWidth',lw); hold on;
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
%plot(t(1:length(theta_ref)),theta_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_p(3,:),'-','Color',C2,'LineWidth',lw); hold on;
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
plot(t,q_p(4,:),'-','Color',C11,'LineWidth',lw); hold on;
plot(t,q_estimate(10,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\phi_{pref}$','$\phi_{pm}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(3,1,2)
%plot(t(1:length(phi_ref)),phi_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_p(5,:),'-','Color',C13,'LineWidth',lw); hold on;
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
plot(t,q_estimate(11,1:length(t)),'--','Color',C14,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\theta_{pref}$','$\theta_{pm}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(3,1,3)
%plot(t(1:length(theta_ref)),theta_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_p(6,:),'-','Color',C2,'LineWidth',lw); hold on;
plot(t,q_estimate(12,1:length(t)),'--','Color',C15,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(c)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\psi_{pref}$','$\psi_{pm}$'},'interpreter','latex','fontsize',fontsizeLegend)

%%%%%%%%

figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(3,1,1)
%plot(t(1:length(F_ref)),F_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,q_p(4,:),'-','Color',C11,'LineWidth',lw); hold on;
plot(t,q_estimate(10,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\phi_{pref}$','$\phi_{pm}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(3,1,2)
%plot(t(1:length(phi_ref)),phi_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_p(5,:),'-','Color',C13,'LineWidth',lw); hold on;
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
plot(t,q_estimate(11,1:length(t)),'--','Color',C14,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\theta_{pref}$','$\theta_{pm}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(3,1,3)
%plot(t(1:length(theta_ref)),theta_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_p(6,:),'-','Color',C2,'LineWidth',lw); hold on;
plot(t,q_estimate(12,1:length(t)),'--','Color',C15,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(c)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\psi_{pref}$','$\psi_{pm}$'},'interpreter','latex','fontsize',fontsizeLegend)