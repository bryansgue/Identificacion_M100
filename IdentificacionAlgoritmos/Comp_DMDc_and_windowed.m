%% VALIDATION FO PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
load('Test4.mat')
load("A_B_values.mat")

 
%% Values init for DMD ONLINE (Discreto)
load("G&P_DMDonline_values_init.mat");

% Windowed DMD Online Value
m=6;

clear tf;
desface = 20;
t = t(1,1:end-desface);
N = length(t);

% REFERENCE SIGNALS FOR WEBOTS
ul_ref = ul_ref(1,1:end-desface);
um_ref = um_ref(1,1:end-desface);
un_ref = un_ref(1,1:end-desface);
w_ref = w_ref(1,1:end-desface);

% REAL SYSTEM VELICITIES FOR WEBOTS DATA
ul = double(ul(1,1:length(ul_ref)));
um = double(um(1,1:length(um_ref)));
un = double(un(1,1:length(un_ref)));
w = double(w(1,1:length(w_ref)));

%% REAL SYSTEM ACCELERATIONS
ulp = [0 , diff(ul)/ts];
ump = [0 , diff(um)/ts];
unp = [0 , diff(un)/ts];
wp = [0 , diff(w)/ts];

%% ACELERATION SYSTEM
ulp = [0 , diff(ul)/ts];
ump = [0 , diff(um)/ts];
unp = [0 , diff(un)/ts];
wp = [0 , diff(w)/ts];

vp = [ulp; ump; unp; wp];

v = [ul; um; un; w];
vp = [ulp; ump; unp; wp];
vref = [ul_ref; um_ref; un_ref; w_ref];


%% For online windowed DMD
Ae = G(:,1:4);
Be = G(:,5:end);
v_estimate1 = v(:,1);
sample = 0;


e1(:,1) = [0;0;0;0];

%% Definition of System Matrcies
Ac = A % From data saved

Bc = B % From data saved
%% SIMULATION DYNAMICS
v_estimate = v(:,1);
for k=1:length(t)
    
    %DMDc
    error_ul_dmd(k) = v(1,k)-v_estimate(1,k);
    error_um_dmd(k) = v(2,k)-v_estimate(2,k);
    error_un_dmd(k) = v(3,k)-v_estimate(3,k);
    error_w_dmd(k)  = v(4,k)-v_estimate(4,k);
    
    vp = (Ac*v_estimate(:,k)+Bc*vref(:,k));
    v_estimate(:, k+1) = v_estimate(:, k) + vp*ts;
    %v_estimate(:, k+1) = A*v_estimate(:,k)+B*vref(:,k);
    
    e1(:,k) = v(:,k) - v_estimate(:, k);
    error1(k) = norm(e1(:,k),2);

    
    %DMD Online
    error_ul_win(k) = v(1,k)-v_estimate1(1,k);
    error_um_win(k) = v(2,k)-v_estimate1(2,k);
    error_un_win(k) = v(3,k)-v_estimate1(3,k);
    error_w_win(k)  = v(4,k)-v_estimate1(4,k);
    
    v_estimate1(:, k+1) = Ae*v_estimate1(:,k)+ Be*vref(:,k);
    rho =1;
    if sample >= m
        [Ae,Be,P,G] = DMD_Online(m,v_estimate1,vref,v,P,G,k,rho);
        sample = 0;   
    end
    sample = sample + 1;
    
    e2(:,k) = v(:,k) - v_estimate1(:, k);
    error2(k) = norm(e2(:,k),2);

    
end


save("Ident_WinDMD_test.mat","v_estimate","v_estimate1","vref","v","t");


ul_err_dmd =  trapz(ts,error_ul_dmd.^2);
um_err_dmd =  trapz(ts,error_um_dmd.^2);
un_err_dmd =  trapz(ts,error_un_dmd.^2);
w_err_dmd =  trapz(ts,error_w_dmd.^2);

ul_err_win =  trapz(ts,error_ul_win.^2);
um_err_win =  trapz(ts,error_um_win.^2);
un_err_win =  trapz(ts,error_un_win.^2);
w_err_win =  trapz(ts,error_w_win.^2);

ERROR = [ul_err_dmd, ul_err_win;...
         um_err_dmd, um_err_win;...
         un_err_dmd, un_err_win;...
         w_err_dmd, w_err_win];
     
save("Comparacion_WinDMD.mat","ERROR","error1","error2","t");

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


figure('Position', [15 15 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(2,2,1)
plot(t(1:length(ul_ref)),ul_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,ul,'-','Color',C11,'LineWidth',lw);
plot(t,v_estimate(1,1:length(t)),'--','Color',C12,'LineWidth',lw);
plot(t,v_estimate1(1,1:length(t)),'--','Color',C3,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\mu_{lref}$','$\mu_{l}$','$\mu_{lm}$','$\mu_{lwindowed}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(2,2,2)
plot(t(1:length(um_ref)),um_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,um,'-','Color',C13,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
plot(t,v_estimate(2,1:length(t)),'--','Color',C14,'LineWidth',lw);
plot(t,v_estimate1(2,1:length(t)),'--','Color',C3,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\mu_{mref}$','$\mu_{m}$','$\mu_{mm}$','$\mu_{mwindowed}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,3)
plot(t(1:length(un_ref)),un_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,un,'-','Color',C2,'LineWidth',lw);
plot(t,v_estimate(3,1:length(t)),'--','Color',C15,'LineWidth',lw);
plot(t,v_estimate1(3,1:length(t)),'--','Color',C3,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(c)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\mu_{nref}$','$\mu_{n}$','$\mu_{nm}$','$\mu_{nwindowed}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,4)
plot(t(1:length(w_ref)),w_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,w,'-','Color',C16,'LineWidth',lw);
plot(t,v_estimate(4,1:length(t)),'--','Color',C17,'LineWidth',lw);
plot(t,v_estimate1(4,1:length(t)),'--','Color',C3,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(d)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\omega_{ref}$','$\omega$','$\omega_{m}$','$\mu_{\omega windowed}$'},'interpreter','latex','fontsize',fontsizeLegend)
%print -dpng Model_dmd_validation
print -depsc Model_dmd_validation

%% USD
figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');
bar(ERROR);
grid on;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel({'$\omega_{ref}$','$\omega$','$\omega_{m}$'},'interpreter','latex','fontsize',fontsizeLabel)
ylabel('$\textrm{ISE}$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
 set(gca,'Xticklabel',{'$ISE_{x}$','$ISE_y$','$ISE_{z}$','$ISE_{\psi}$'})
legend({'$\textrm{DMDc}$','$\textrm{Windowed DMDc}$'},'interpreter','latex','fontsize',fontsizeLegend)


figure('Position', [15 15 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

