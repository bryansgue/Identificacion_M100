%% Code to show the results of the Inverse kinematic controller

% Clean variables
clc, clear all, close all;
 
% Load variables of the system
% load("Identification.mat");
%load("Validation.mat");


load("IdentDMD_test.mat");


t_iden = t;
%Vel Ref
ul_ref_iden = v_ref(1,:);
um_ref_iden = v_ref(2,:);
un_ref_iden = v_ref(3,:);
w_ref_iden = v_ref(4,:);
%Vel reales
ul_iden = v_real(1,:);
um_iden = v_real(2,:);
un_iden = v_real(3,:);
w_iden = v_real(4,:);
%Vel Modelo
v_estimate_inde = v_estimate;


%tiempo








% Figure propert%% Figures
lw = 1; % linewidth 1
lwV = 2; % linewidth 2
fontsizeLabel = 11; %11
fontsizeLegend = 11;
fontsizeTicks = 11;
fontsizeTitel = 11;
sizeX = 1300; % size figure
sizeY = 750; % size figure

% color propreties
c1 = [80, 81, 79]/255;
c2 = [244, 213, 141]/255;
c3 = [242, 95, 92]/255;
c4 = [112, 141, 129]/255;

C18 = [0 0 0];
c5 = [130, 37, 37]/255;
c6 = [205, 167, 37]/255;
c7 = [81, 115, 180]/255;

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

%% New color slection

figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

%% Figure 1a
axes('Position',[0.04 0.79 .45 .17]);
%% Data generation
ul_iden = line(t_iden,ul_iden);
set(ul_iden, 'LineStyle', '-', 'Color', C11, 'LineWidth', 1.1*lw);
ul_ref_iden = line(t_iden(1:length(ul_ref_iden)),ul_ref_iden);
set(ul_ref_iden, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);
ulm_iden = line(t_iden,v_estimate_inde(1,1:length(t_iden)));
set(ulm_iden, 'LineStyle', '--', 'Color', C12, 'LineWidth', 1.1*lw);

%% Title of the image
hTitle_1 = title({'Identificaci\''on $\nu_{l}$'},'fontsize',12,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_1 = legend([ul_ref_iden, ul_iden, ulm_iden],{'$\mu_{l}$','$\nu_{l}$','$\nu_{l_m}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.1*fontsizeTicks)
%% Figure properties
ax_1 = gca;
ax_1.Box = 'on';
ax_1.BoxStyle = 'full';
ax_1.TickLength = [0.01;0.01];
ax_1.TickDirMode = 'auto';
ax_1.XTickLabel = [];
ax_1.YMinorTick = 'on';
ax_1.XMinorTick = 'on';
ax_1.XMinorGrid = 'on';
ax_1.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_1.MinorGridAlpha = 0.15;
ax_1.LineWidth = 0.8;
ax_1.XLim = [0 t_iden(end)];

%% Figure 2
axes('Position',[0.52+0.02 0.79 .45 .17]);
%% Data generation
um_iden = line(t_iden,um_iden);
set(um_iden, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
um_ref_iden = line(t_iden(1:length(um_ref_iden)),um_ref_iden);
set(um_ref_iden, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);
umm_iden = line(t_iden,v_estimate_inde(2,1:length(t_iden)));
set(umm_iden, 'LineStyle', '--', 'Color', C14, 'LineWidth', 1.1*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_2 = title({'Identificaci\''on $\nu_{m}$'},'fontsize',12,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_2 = legend([um_ref_iden, um_iden, umm_iden],{'$\mu_{m}$','$\nu_{m}$','$\nu_{m_m}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1*fontsizeTicks)
%% Figure properties
ax_2 = gca;
ax_2.Box = 'on';
ax_2.BoxStyle = 'full';
ax_2.TickLength = [0.01;0.01];
ax_2.TickDirMode = 'auto';
ax_2.XTickLabel = [];
ax_2.YMinorTick = 'on';
ax_2.XMinorTick = 'on';
ax_2.XMinorGrid = 'on';
ax_2.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_2.MinorGridAlpha = 0.15;
ax_2.LineWidth = 0.8;
ax_2.XLim = [0 t_iden(end)];

%% Figure 3
axes('Position',[0.04 0.37-0.05  .45 .17]);
%% Data generation
un_iden = line(t_iden,un_iden);
set(un_iden, 'LineStyle', '-', 'Color', C2, 'LineWidth', 1.1*lw);
un_ref_iden = line(t_iden(1:length(un_ref_iden)),un_ref_iden);
set(un_ref_iden, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);
unm_iden = line(t_iden,v_estimate_inde(3,1:length(t_iden)));
set(unm_iden, 'LineStyle', '--', 'Color', C15, 'LineWidth', 1.1*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_2 = title({'Identificaci\''on $\nu_{n}$'},'fontsize',12,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_3 = legend([un_ref_iden, un_iden, unm_iden],{'$\mu_{n}$','$\nu_{n}$','$\nu_{n_m}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1*fontsizeTicks)
%% Figure properties
ax_3 = gca;
ax_3.Box = 'on';
ax_3.BoxStyle = 'full';
ax_3.TickLength = [0.01;0.01];
ax_3.TickDirMode = 'auto';
ax_3.XTickLabel = [];
ax_3.YMinorTick = 'on';
ax_3.XMinorTick = 'on';
ax_3.XMinorGrid = 'on';
ax_3.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_3.MinorGridAlpha = 0.15;
ax_3.LineWidth = 0.8;
ax_3.XLim = [0 t_iden(end)];

%% Figure 4
axes('Position',[0.52+0.02 0.37-0.05 .45 .17]);
%% Data generation
w_iden = line(t_iden,w_iden);
set(w_iden, 'LineStyle', '-', 'Color', C16, 'LineWidth', 1.1*lw);
w_ref_iden = line(t_iden(1:length(w_ref_iden)),w_ref_iden);
set(w_ref_iden, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);
w_m_iden = line(t_iden,v_estimate_inde(4,1:length(t_iden)));
set(w_m_iden, 'LineStyle', '--', 'Color', C17, 'LineWidth', 1.1*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_2 = title({'Identificaci\''on $\nu_{\omega}$'},'fontsize',12,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',9,'interpreter','latex','Color',C18);
ylabel('$[rad/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([w_ref_iden, w_iden, w_m_iden],{'$\mu_{\omega}$','$\nu_{\omega}$','$\nu_{\omega_m}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1*fontsizeTicks)
%% Figure properties
ax_4 = gca;
ax_4.Box = 'on';
ax_4.BoxStyle = 'full';
ax_4.TickLength = [0.01;0.01];
ax_4.TickDirMode = 'auto';
ax_4.XTickLabel = [];
ax_4.YMinorTick = 'on';
ax_4.XMinorTick = 'on';
ax_4.XMinorGrid = 'on';
ax_4.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;
ax_4.XLim = [0 t_iden(end)];
% % 
%%
load("Ident_WinDMD_test.mat");

t_iden = t;
%Vel Ref
ul_ref_iden = vref(1,:);
um_ref_iden = vref(2,:);
un_ref_iden = vref(3,:);
w_ref_iden = vref(4,:);
%Vel reales
ul_iden = v(1,:);
um_iden = v(2,:);
un_iden = v(3,:);
w_iden = v(4,:);
%Vel Modelo
v_estimate_inde = v_estimate;
v_estimate_inde1 = v_estimate1;
%%
%% Figure 1b
axes('Position',[0.04 0.58 .45 .17]);

%% Data generation
ul_iden = line(t_iden,ul_iden);
set(ul_iden, 'LineStyle', '-', 'Color', C11, 'LineWidth', 1.3*lw);
ul_ref_iden = line(t_iden(1:length(ul_ref_iden)),ul_ref_iden);
set(ul_ref_iden, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);
ulm_iden = line(t_iden,v_estimate_inde(1,1:length(t_iden)));
set(ulm_iden, 'LineStyle', '--', 'Color', C12, 'LineWidth', 1.1*lw);
ulwin_iden = line(t_iden,v_estimate_inde1(1,1:length(t_iden)));
set(ulwin_iden, 'LineStyle', '-.', 'Color', C3, 'LineWidth', 1.3*lw);

%% Title of the image
hTitle_1 = title({'Validaci\''on $\nu_{l}$'},'fontsize',12,'interpreter','latex','Color',C18);
xlabel('$\textrm{Tiempo}[s]$','fontsize',9,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_1 = legend([ul_ref_iden, ul_iden, ulm_iden],{'$\mu_{l}$','$\nu_{l}$','$\nu_{l_m}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.1*fontsizeTicks)
%% Figure properties
ax_1 = gca;
ax_1.Box = 'on';
ax_1.BoxStyle = 'full';
ax_1.TickLength = [0.01;0.01];
ax_1.TickDirMode = 'auto';
%ax_1.XTickLabel = [];
ax_1.YMinorTick = 'on';
ax_1.XMinorTick = 'on';
ax_1.XMinorGrid = 'on';
ax_1.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_1.MinorGridAlpha = 0.15;
ax_1.LineWidth = 0.8;
ax_1.XLim = [0 t_iden(end)];

%% Figure 2
axes('Position',[0.52+0.02 0.58 .45 .17]);
%% Data generation
um_iden = line(t_iden,um_iden);
set(um_iden, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.3*lw);
um_ref_iden = line(t_iden(1:length(um_ref_iden)),um_ref_iden);
set(um_ref_iden, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);
umm_iden = line(t_iden,v_estimate_inde(2,1:length(t_iden)));
set(umm_iden, 'LineStyle', '--', 'Color', C14, 'LineWidth', 1.1*lw);
umwin_iden = line(t_iden,v_estimate_inde1(2,1:length(t_iden)));
set(umwin_iden, 'LineStyle', '-.', 'Color', C8, 'LineWidth', 1.3*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_2 = title({'Validaci\''on $\nu_{m}$'},'fontsize',12,'interpreter','latex','Color',C18);
xlabel('$\textrm{Tiempo}[s]$','fontsize',9,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_2 = legend([um_ref_iden, um_iden, umm_iden],{'$\mu_{m}$','$\nu_{m}$','$\nu_{m_m}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1*fontsizeTicks)
%% Figure properties
ax_2 = gca;
ax_2.Box = 'on';
ax_2.BoxStyle = 'full';
ax_2.TickLength = [0.01;0.01];
ax_2.TickDirMode = 'auto';
%ax_2.XTickLabel = [];
ax_2.YMinorTick = 'on';
ax_2.XMinorTick = 'on';
ax_2.XMinorGrid = 'on';
ax_2.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_2.MinorGridAlpha = 0.15;
ax_2.LineWidth = 0.8;
ax_2.XLim = [0 t_iden(end)];

%% Figure 3

axes('Position',[0.04 0.16-0.05 .45 .17]);
%% Data generation
un_iden = line(t_iden,un_iden);
set(un_iden, 'LineStyle', '-', 'Color', C2, 'LineWidth', 1.3*lw);
un_ref_iden = line(t_iden(1:length(un_ref_iden)),un_ref_iden);
set(un_ref_iden, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);
unm_iden = line(t_iden,v_estimate_inde(3,1:length(t_iden)));
set(unm_iden, 'LineStyle', '--', 'Color', C15, 'LineWidth', 1.1*lw);
unwin_iden = line(t_iden,v_estimate_inde1(3,1:length(t_iden)));
set(unwin_iden, 'LineStyle', '-.', 'Color', C12, 'LineWidth', 1.3*lw)

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_2 = title({'Validaci\''on $\nu_{n}$'},'fontsize',12,'interpreter','latex','Color',C18);
xlabel('$\textrm{Tiempo}[s]$','fontsize',9,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_3 = legend([un_ref_iden, un_iden, unm_iden],{'$\mu_{n}$','$\nu_{n}$','$\nu_{n_m}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1*fontsizeTicks)
%% Figure properties
ax_3 = gca;
ax_3.Box = 'on';
ax_3.BoxStyle = 'full';
ax_3.TickLength = [0.01;0.01];
ax_3.TickDirMode = 'auto';
%ax_3.XTickLabel = [];
ax_3.YMinorTick = 'on';
ax_3.XMinorTick = 'on';
ax_3.XMinorGrid = 'on';
ax_3.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_3.MinorGridAlpha = 0.15;
ax_3.LineWidth = 0.8;
ax_3.XLim = [0 t_iden(end)];

%% Figure 4
axes('Position',[0.52+0.02 0.16-0.05 .45 .17]);
%% Data generation
w_iden = line(t_iden,w_iden);
set(w_iden, 'LineStyle', '-', 'Color', C16, 'LineWidth', 1.3*lw);
w_ref_iden = line(t_iden(1:length(w_ref_iden)),w_ref_iden);
set(w_ref_iden, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);
w_m_iden = line(t_iden,v_estimate_inde(4,1:length(t_iden)));
set(w_m_iden, 'LineStyle', '--', 'Color', C17, 'LineWidth', 1.1*lw);
wwin_iden = line(t_iden,v_estimate_inde1(4,1:length(t_iden)));
set(wwin_iden, 'LineStyle', '-.', 'Color', C15, 'LineWidth', 1.3*lw)

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_2 = title({'Validaci\''on $\nu_{\omega}$'},'fontsize',12,'interpreter','latex','Color',C18);
xlabel('$\textrm{Tiempo}[s]$','fontsize',9,'interpreter','latex','Color',C18);
ylabel('$[rad/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([w_ref_iden, w_iden, w_m_iden],{'$\mu_{\omega}$','$\nu_{\omega}$','$\nu_{\omega_m}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1*fontsizeTicks)
%% Figure properties
ax_4 = gca;
ax_4.Box = 'on';
ax_4.BoxStyle = 'full';
ax_4.TickLength = [0.01;0.01];
ax_4.TickDirMode = 'auto';
ax_4.YMinorTick = 'on';
ax_4.XMinorTick = 'on';
ax_4.XMinorGrid = 'on';
ax_4.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;
ax_4.XLim = [0 t_iden(end)];
% % 

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Validation_identification_V3.pdf -q101