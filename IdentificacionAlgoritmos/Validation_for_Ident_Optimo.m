%% VALIDATION FO PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
load('Test3.mat')
load("chi_values.mat");
clear tf;
desface = 0;
t = t(1,1:end-desface);

N = length(t);
%% REFERENCE SIGNALS

switch 1
    case 1
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
    case 2
        % REFERENCE SIGNALS FOR MATRICE DATA
        ul_ref = double(vf_ref(1,1:end-desface));
        um_ref = double(vl_ref(1,1:end-desface));
        un_ref = double(ve_ref(1,1:end-desface));
        w_ref = double(w_ref(1,1:end-desface));

        % REAL SYSTEM VELICITIES FOR MATRICE DATA
        ul = double(vf(1,1:length(ul_ref)));
        um = double(vl(1,1:length(um_ref)));
        un = double(ve(1,1:length(un_ref)));
        w = double(w(1,1:length(w_ref)));
    otherwise
        disp('other value')
end

% ul_ref = vf_ref;
% um_ref = vl_ref;
% un_ref = ve_ref;
% w_ref = w_ref;

a =0;
b=0;
L=[a;b];

%% REAL SYSTEM VELICITIES
% ul = double(vf(1,1:length(ul_ref)));
% um = double(vl(1,1:length(um_ref))+eps*randn(size(ul)));
% un = double(ve(1,1:length(un_ref))+eps*randn(size(ul)));
% w = double(w(1,1:length(w_ref))+eps*randn(size(ul)));


%% ACELERATION SYSTEM
ulp = [0 , diff(ul)/ts];
ump = [0 , diff(um)/ts];
unp = [0 , diff(un)/ts];
wp = [0 , diff(w)/ts];

vp = [ulp; ump; unp; wp];
v = [ul; um; un; w];
v_ref = [ul_ref; um_ref; un_ref; w_ref];

x = chi;
%% SIMULATION DYNAMICS
v_estimate = v(:,1);
for k=1:length(t)
    v_estimate(:, k+1) = dynamic_model_for_sim(x, v_estimate(:,k), v_ref(:,k), psi(k), L,ts);
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


% figure
% set(gcf, 'PaperUnits', 'inches');
% %set(gcf, 'PaperSize', [8.5 11]);
% set(gcf, 'PaperPositionMode', 'manual');
% box on
% subplot(2,1,1)
% plot(t(1:length(ul_ref)),ul_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
% %plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
% plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% grid minor;
% set(gca,'ticklabelinterpreter','latex',...
%         'fontsize',fontsizeTicks)
% %xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\mu_{lref}$','$\mu_{l}$'},'interpreter','latex','fontsize',fontsizeLegend)
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% subplot(2,1,2)
% plot(t,ul,'-','Color',C11,'LineWidth',lw*1.2); hold on
% plot(t,v_estimate(1,1:length(t)),'--','Color',C12,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%     'fontsize',fontsizeTicks)
% title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% %title({'a) SEIR system identification: DMD vs. SINDy';''},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\mu_{l}$','$\mu_{lm}$'},'interpreter','latex','fontsize',fontsizeLegend)
% print -dpng Model_dmd_ul
% print -depsc Model_dmd_ul
% 
% figure
% set(gcf, 'PaperUnits', 'inches');
% %set(gcf, 'PaperSize', [8.5 11]);
% set(gcf, 'PaperPositionMode', 'manual');
% box on
% subplot(2,1,1)
% plot(t(1:length(um_ref)),um_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
% plot(t,um,'-','Color',C13,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%         'fontsize',fontsizeTicks)
% %xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\mu_{mref}$','$\mu_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% subplot(2,1,2)
% plot(t,um,'-','Color',C13,'LineWidth',lw*1.2); hold on
% plot(t,v_estimate(2,1:length(t)),'--','Color',C14,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%     'fontsize',fontsizeTicks)
% title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% %title({'a) SEIR system identification: DMD vs. SINDy';''},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\mu_{m}$','$\mu_{mm}$'},'interpreter','latex','fontsize',fontsizeLegend)
% print -dpng Model_dmd_um
% print -depsc Model_dmd_um
% 
% 
% figure
% set(gcf, 'PaperUnits', 'inches');
% %set(gcf, 'PaperSize', [8.5 11]);
% set(gcf, 'PaperPositionMode', 'manual');
% box on
% subplot(2,1,1)
% plot(t(1:length(w_ref)),w_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
% plot(t,w,'-','Color',C16,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%         'fontsize',fontsizeTicks)
% %xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\omega_{ref}$','$\omega$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% subplot(2,1,2)
% plot(t,w,'-','Color',C16,'LineWidth',lw*1.2); hold on
% plot(t,v_estimate(4,1:length(t)),'--','Color',C17,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%     'fontsize',fontsizeTicks)
% title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
% %title({'a) SEIR system identification: DMD vs. SINDy';''},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\omega$','$\omega_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)
% print -dpng Model_dmd_omega
% print -depsc Model_dmd_omega
% 
% figure
% set(gcf, 'PaperUnits', 'inches');
% %set(gcf, 'PaperSize', [8.5 11]);
% set(gcf, 'PaperPositionMode', 'manual');
% box on
% subplot(2,1,1)
% plot(t(1:length(un_ref)),un_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
% plot(t,un,'-','Color',C2,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%         'fontsize',fontsizeTicks)
% %xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\mu_{nref}$','$\mu_{n}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% subplot(2,1,2)
% plot(t,un,'-','Color',C2,'LineWidth',lw*1.2); hold on
% plot(t,v_estimate(3,1:length(t)),'--','Color',C15,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%     'fontsize',fontsizeTicks)
% title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% %title({'a) SEIR system identification: DMD vs. SINDy';''},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\mu_{n}$','$\mu_{nm}$'},'interpreter','latex','fontsize',fontsizeLegend)
% print -dpng Model_dmd_un
% print -depsc Model_dmd_un
% 
% figure('Position', [10 10 sizeX sizeY])
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [8.5 11]);
% set(gcf, 'PaperPositionMode', 'manual');
% box on
% subplot(4,2,1)
% plot(t(1:length(ul_ref)),ul_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
% %plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
% plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% grid minor;
% set(gca,'ticklabelinterpreter','latex',...
%         'fontsize',fontsizeTicks)
% %xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\mu_{lref}$','$\mu_{l}$'},'interpreter','latex','fontsize',fontsizeLegend)
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% subplot(4,2,3)
% plot(t,ul,'-','Color',C11,'LineWidth',lw*1.2); hold on
% plot(t,v_estimate(1,1:length(t)),'--','Color',C12,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%     'fontsize',fontsizeTicks)
% %title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% %xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% %title({'a) SEIR system identification: DMD vs. SINDy';''},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\mu_{l}$','$\mu_{lm}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% box on
% subplot(4,2,2)
% plot(t(1:length(um_ref)),um_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
% plot(t,um,'-','Color',C13,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%         'fontsize',fontsizeTicks)
% %xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\mu_{mref}$','$\mu_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% subplot(4,2,4)
% plot(t,um,'-','Color',C13,'LineWidth',lw*1.2); hold on
% plot(t,v_estimate(2,1:length(t)),'--','Color',C14,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%     'fontsize',fontsizeTicks)
% %title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% %xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% %title({'a) SEIR system identification: DMD vs. SINDy';''},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\mu_{m}$','$\mu_{mm}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% box on
% subplot(4,2,5)
% plot(t(1:length(un_ref)),un_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
% plot(t,un,'-','Color',[237,80,186]/255,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%         'fontsize',fontsizeTicks)
% %xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(c)'},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\mu_{nref}$','$\mu_{n}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% subplot(4,2,7)
% plot(t,un,'-','Color',[237,80,186]/255,'LineWidth',lw*1.2); hold on
% plot(t,v_estimate(3,1:length(t)),'--','Color',[35,211,217]/255,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%     'fontsize',fontsizeTicks)
% %title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% xlabel('$\textrm{time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
% %title({'a) SEIR system identification: DMD vs. SINDy';''},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\mu_{n}$','$\mu_{nm}$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% box on
% subplot(4,2,6)
% plot(t(1:length(w_ref)),w_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
% plot(t,w,'-','Color',C16,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%         'fontsize',fontsizeTicks)
% %xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
% title({'(d)'},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\omega_{ref}$','$\omega$'},'interpreter','latex','fontsize',fontsizeLegend)
% 
% subplot(4,2,8)
% plot(t,w,'-','Color',C16,'LineWidth',lw*1.2); hold on
% plot(t,v_estimate(4,1:length(t)),'--','Color',C17,'LineWidth',lw);
% %plot(t,ul,'-','Color',C2,'LineWidth',lw);
% grid minor;
% 
% set(gca,'ticklabelinterpreter','latex',...
%     'fontsize',fontsizeTicks)
% %title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% xlabel('$\textrm{time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
% ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
% %title({'a) SEIR system identification: DMD vs. SINDy';''},'fontsize',fontsizeTitel,'interpreter','latex')
% % set(gca,'Xticklabel',[])
% legend({'$\omega$','$\omega_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)
% print -dpng Model_dmd_completo
% print -depsc Model_dmd_completo

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
plot(t,w,'-','Color',C16,'LineWidth',lw);
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
%print -dpng Model_optimization_validation
print -depsc Model_optimization_validation

