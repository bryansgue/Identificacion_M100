%% ESTIMATION FO PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES

load('Test4.mat')

ts = 0.1;       % Tiempo de muestreo
tfin = 300;      % Tiempo de simulaciÃ³n
t = 0:ts:tfin;
clear tf;
polyorder = 2;    % Library terms polynomial order
usesine   = 0;    % Using sine functions in library


desface = 0;
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
%Y = vp(:,2:end);
P = inv(X_k*X_k');
G = Y*pinv(X_k)
%G = Y*X_k'*inv(X_k*X_k')
Ae = G(:,1:4);
Be = G(:,5:end);
save("G&P_DMDonline_values_init.mat","G","P");
