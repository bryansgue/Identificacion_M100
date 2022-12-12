%% ESTIMATION FO PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES

load('Test1.mat')
ts = 0.05;       % Tiempo de muestreo
tfin = 300;      % Tiempo de simulaciÃ³n
t = 0:ts:tfin;
clear tf;
polyorder = 2;    % Library terms polynomial order
usesine   = 0;    % Using sine functions in library
n = 4;

desface = 80;
t = t(1,1:end-desface);
%% REFERENCE SIGNALS
ul_ref = ul_ref(1,1:end-desface);
um_ref = um_ref(1,1:end-desface);
un_ref = un_ref(1,1:end-desface);
w_ref = w_ref(1,1:end-desface);

%% REAL SYSTEM VELICITIES
ul = double(xu_p(1,1:length(ul_ref)));
um = double(yu_p(1,1:length(um_ref)));
un = double(zu_p(1,1:length(un_ref)));
w = double(w(1,1:length(w_ref)));


%% REAL SYSTEM ACCELERATIONS
ulp = [0 , diff(ul)/ts];
ump = [0 , diff(um)/ts];
unp = [0 , diff(un)/ts];
wp = [0 , diff(w)/ts];


v_real = [ul; um; un; w];
vp = [ulp; ump; unp; wp];
vc = [ul_ref; um_ref; un_ref; w_ref];

v_estimate1 = v_real(:,1);

%% Estimacion inicial DMD online discreto
X_k = [v_real(:,1:end-1);
       vc(:,1:end-1)];
Y = v_real(:,2:end);
P = inv(X_k*X_k')
G = Y*X_k'*inv(X_k*X_k')
Ae = G(:,1:4);
Be = G(:,5:end);
save("/home/bryansgue/Doctorado/Matlab/UAV/G&P_DMDonline_values_init.mat","G","P");
