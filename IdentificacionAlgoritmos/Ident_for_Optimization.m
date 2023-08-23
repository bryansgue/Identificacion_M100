%% IDENTIFICATION OF PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
load('M100_ident_sim_3.mat')
clear tf;
desface = 0;
t = t(1,1:end-desface);
N = length(t);

landa = 10;%lambda
F1=tf(landa,[1 landa]);

%% U_REF
ul_ref = v_ref(1,1:end-desface);
um_ref = v_ref(2,1:end-desface);
un_ref = v_ref(3,1:end-desface);
w_ref  = v_ref(4,1:end-desface);

u_ref = [ul_ref; um_ref; un_ref; w_ref];

ul_ref_f=lsim(F1,ul_ref,t)';
um_ref_f=lsim(F1,um_ref,t)';
un_ref_f=lsim(F1,un_ref,t)';
w_ref_f=lsim(F1,w_ref,t)';

u_ref_f = [ul_ref_f; um_ref_f; un_ref_f; w_ref_f];

%% V 
vx = double(v(1,1:length(ul_ref)));
vy = double(v(2,1:length(um_ref)));
vz = double(v(3,1:length(un_ref)));

v = [vx; vy; vz];

vx_f=lsim(F1,vx,t)';
vy_f=lsim(F1,vy,t)';
vz_f=lsim(F1,vz,t)';

v_f = [vx_f; vy_f; vz_f];

%% V_p 

for k=1:length(t)
    if k>1
        vx_p(k)=(vx(k)- vx(k-1))/ts;
        vy_p(k)=(vy(k)- vy(k-1))/ts;
        vz_p(k)=(vz(k)- vz(k-1))/ts;     
    else
        vx_p(k)=0;
        vy_p(k)=0;
        vz_p(k)=0;
        
    end
end

v_p = [vx_p; vy_p; vz_p];

vx_p_f=lsim(F1,vx_p,t)';
vy_p_f=lsim(F1,vy_p,t)';
vz_p_f=lsim(F1,vz_p,t)';

v_p_f = [vx_p_f; vy_p_f; vz_p_f];

%% U 
% 
% for k=1:length(t)-1
%      R = QuatToRot(quat(:,k))
%      u(:,k) = inv(R)*v(:,k);
% end 

ul = double(u(1,1:length(ul_ref)));
um = double(u(2,1:length(um_ref)));
un = double(u(3,1:length(un_ref)));

u = [ul; um; un];

ul_f=lsim(F1,ul,t)';
um_f=lsim(F1,um,t)';
un_f=lsim(F1,un,t)';

u_f = [ul_f; um_f; un_f];

%% U_p 

for k=1:length(t)
    if k>1
        ul_p(k)=(ul(k)- ul(k-1))/ts;
        um_p(k)=(um(k)- um(k-1))/ts;
        un_p(k)=(un(k)- un(k-1))/ts;     
    else
        ul_p(k)=0;
        um_p(k)=0;
        un_p(k)=0;
        
    end
end

u_p = [ul_p; um_p; un_p];

ul_p_f=lsim(F1,ul_p,t)';
um_p_f=lsim(F1,um_p,t)';
un_p_f=lsim(F1,un_p,t)';

u_p_f = [ul_p_f; um_p_f; un_p_f];

%% OMEGA
p = double(omega(1,1:length(ul_ref)));
q = double(omega(2,1:length(um_ref)));
r = double(omega(3,1:length(un_ref)));

omega = [p;q;r];

p_f=lsim(F1,p,t)';
q_f=lsim(F1,q,t)';
r_f=lsim(F1,r,t)';

omega_f = [p_f; q_f; r_f];

%% QUATERNIOS
qw = double(quat(1,1:length(ul_ref)));
qx = double(quat(2,1:length(ul_ref)));
qy = double(quat(3,1:length(ul_ref)));
qz = double(quat(4,1:length(ul_ref)));

quat = [qw;qx;qy;qz];

qw_f=lsim(F1,qw,t)';
qx_f=lsim(F1,qx,t)';
qy_f=lsim(F1,qy,t)';
qz_f=lsim(F1,qz,t)';

quat_f = [qw_f; qx_f; qy_f; qz_f];

%% Omega_p

for k=1:length(t)
    if k>1
        p_p(k)=(p(k)- p(k-1))/ts;
        q_p(k)=(q(k)- q(k-1))/ts;
        r_p(k)=(r(k)- r(k-1))/ts;
        %wp(k) =(w(k)- w(k-1))/ts;
    else
        p_p(k)=0;
        q_p(k)=0;
        r_p(k)=0;
        %wp(k) =0;
    end
end

omega_p = [p_p; q_p; r_p];

p_p_f=lsim(F1,p_p,t)';
q_p_f=lsim(F1,q_p,t)';
r_p_f=lsim(F1,r_p,t)';

omega_p_f = [p_p_f; q_p_f; r_p_f];

a = 0;
b = 0;

L = [a;b];

%% Parametros del optimizador
options = optimset('Display','iter',...
    'TolFun', 1e-8,...
    'MaxIter', 10000,...
    'Algorithm', 'active-set',...
    'FinDiffType', 'forward',...
    'RelLineSrchBnd', [],...
    'RelLineSrchBndDuration', 1,...
    'TolConSQP', 1e-6);
x0=zeros(1,18);
f_obj1 = @(x) funcion_costo(x, u_ref_f, u_p_f, u_f, omega,omega_p, N, L);
x = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
chi = x;
%save("/home/bryansgue/Doctorado/Matlab/UAV/DynamicControllers/chi_values.mat","chi");

%save("/home/bryan/Proyectos_Matlab/Doctorado/Matlab/IdentificacionM100/IdentificacionUAV/chi_values.mat","chi");
save("chi_values.mat","chi");

%% SIMULATION DYNAMICS
v_estimate = [v(:,1);omega(3,1)];
for k=1:length(t)
    v_estimate(:, k+1) = dynamic_model_for_sim(x, v_estimate(:,k), v_ref(:,k), psi(k), L, ts);
end

%% Parameters fancy plots
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


figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(2,2,1)
plot(t(1:length(ul_ref)),ul_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,ul,'-','Color',C11,'LineWidth',lw);
plot(t,v_estimate(1,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\mu_{lref}$','$\mu_{l}$','$\mu_{lm}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(2,2,2)
plot(t(1:length(um_ref)),um_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,um,'-','Color',C13,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
plot(t,v_estimate(2,1:length(t)),'--','Color',C14,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
    'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\mu_{mref}$','$\mu_{m}$','$\mu_{mm}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,3)
plot(t(1:length(un_ref)),un_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,un,'-','Color',C2,'LineWidth',lw);
plot(t,v_estimate(3,1:length(t)),'--','Color',C15,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
    'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(c)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\mu_{nref}$','$\mu_{n}$','$\mu_{nm}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,4)
plot(t(1:length(w_ref)),w_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,omega(3),'-','Color',C16,'LineWidth',lw);
plot(t,v_estimate(4,1:length(t)),'--','Color',C17,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
    'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(d)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\omega_{ref}$','$\omega$','$\omega_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)
%print -dpng Model_optimization_identification
print -depsc Model_optimization_identification
