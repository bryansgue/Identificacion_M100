% Inicializacion
tic
clc; clear all; close all; warning off % Inicializacion
ts = 1/30;       
tfin =20;     
t = 0:ts:tfin;

%% Inicializa Nodo ROS
rosshutdown
setenv('ROS_MASTER_URI','http://192.168.88.85:11311')
setenv('ROS_IP','192.168.88.104')
rosinit

%% CONSTANTS VALUES OF THE ROBOT
a = 0.0; 
b = 0.0;
c = 0.0;
L = [a, b, c];

%% 1) PUBLISHER TOPICS & MSG ROS - UAV M100

[velControl_topic,velControl_msg] = rospublisher('/m100/velocityControl','geometry_msgs/TwistStamped');
u_ref = [0.37, 0.0, 0.0, 0.0];
send_velocities_euler(velControl_topic, velControl_msg, u_ref);

%% 2) Suscriber TOPICS & MSG ROS - UAV M100

odomSub = rossubscriber('/dji_sdk/odometry');


for i=1:10
    tic
    Datos = odometryUAV(odomSub);
    
    while toc<ts
    end
    
end
disp("Running...")

[h(:,1),euler(:,1),v(:,1),omega(:,1),quat(:,1)] = odometryUAV(odomSub);

euler_p(:,1) = Euler_p(omega(:,1),euler(:,1)); 
R(:,:,1) = quatToRot(quat(:,1));

n=12; % Seleccion UAV
n_chr = int2str(n);

w_ref_active = true;
banda = 60;
u_ref = Velocities_data_euler_input(n,t,w_ref_active, banda);

%% Matriz de rotacion


v_p(:,1) = [0;0;0];
%% Comienza el programa
for k=1:length(t)
    tic
    
%% SEND VALUES OF CONTROL ROBOT  - UAV M100

    send_velocities_euler(velControl_topic, velControl_msg, u_ref(:,k));
    
    
    while toc<ts
    end
    
    %% 3) Odometria del UAV
    [h(:,k+1),euler(:,k+1),v(:,k+1),omega(:,k+1),quat(:,k+1)] = odometryUAV(odomSub);
    
    euler_p(:,k+1) = Euler_p(omega(:,k+1),euler(:,k+1));
      
    R(:,:,k) = quatToRot(quat(:,k+1));

    u(:,k+1) = inv(R(:,:,k))*v(:,k+1);
                     
                    
% 3) Tiempo de mÃ¡quina   
     dt(k) = toc;
 

end
%%




send_velocities(velControl_topic, velControl_msg, [0, 0, 0, 0]);
s0 = "IdentificacionAlgoritmos/";
s1 = 'full_ident_M100_';
s2 = '.mat';
s = strcat(s0,s1,n_chr,s2)


 save(s,"h","euler","euler_p","v","ts","omega","R","quat","t","u_ref")

%% %% Parameters fancy plots
% define plot properties
lw = 2; % linewidth 1
lwV = 2; % linewidth 2
fontsizeLabel = 9; %11
fontsizeLegend = 20;
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

%% u vs v

figure('Position', [15 15 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(3,1,1)

plot(t(1:length(u_ref(2,:))),u_ref(2,:),'-','Color',C1,'LineWidth',lw*1);hold on

plot(t(1:length(euler(1,1:end-1))),euler(1,1:end-1),'-','Color',C2,'LineWidth',lw*1);hold on

grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\phi_{ref}$','$\phi$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

%%
% EULER
subplot(3,1,2)

plot(t(1:length(u_ref(3,:))),u_ref(3,:),'-','Color',C3,'LineWidth',lw*1);hold on

plot(t(1:length(euler(2,1:end-1))),euler(2,1:end-1),'-','Color',C4,'LineWidth',lw*1);hold on

grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\theta_{ref}$','$\theta$'},'interpreter','latex','fontsize',fontsizeLegend)
%%
% OMEGA
subplot(3,1,3)

plot(t(1:length(u_ref(4,:))),u_ref(4,:),'-','Color',C3,'LineWidth',lw*1);hold on

plot(t(1:length(omega(3,1:end-1))),omega(3,1:end-1),'-','Color',C4,'LineWidth',lw*1);hold on

grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\psi_{p_{ref}}$','$\psi_p$'},'interpreter','latex','fontsize',fontsizeLegend)

% u_p

%%






