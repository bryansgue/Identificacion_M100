% Inicializacion
tic
clc; clear all; close all; warning off % Inicializacion
ts = 1/30;       
tfin =30;     
t = 0:ts:tfin;

%% Inicializa Nodo ROS
rosshutdown
setenv('ROS_MASTER_URI','http://192.168.88.223:11311')
setenv('ROS_IP','192.168.88.104')
rosinit

%% CONSTANTS VALUES OF THE ROBOT
a = 0.0; 
b = 0.0;
c = 0.0;
L = [a, b, c];

%% 1) PUBLISHER TOPICS & MSG ROS - UAV M100

[velControl_topic,velControl_msg] = rospublisher('/m100/velocityControl','geometry_msgs/TwistStamped');
u_ref = [0.0, 0.0, 0.0, 0.0];
send_velocities(velControl_topic, velControl_msg, u_ref);

%% 2) Suscriber TOPICS & MSG ROS - UAV M100

odomSub = rossubscriber('/dji_sdk/odometry');

[h(:,1),euler(:,1),v(:,1),omega(:,1),quat(:,1)] = odometryUAV(odomSub);

euler_p(:,1) = [0;0;0]; 

n=3; % Seleccion UAV
n_chr = int2str(n);


w_ref_active = true;
v_ref = Velocities_data(n,t,w_ref_active );

%% Matriz de rotacion

v_p(:,1) = [0;0;0];
%% Comienza el programa
for k=1:length(t)
    
    tic   
    % SEND VALUES OF CONTROL ROBOT  - UAV M100
    send_velocities(velControl_topic, velControl_msg, v_ref(:,k));
    
    %% 3) Odometria del UAV
    
    [h(:,k+1),euler(:,k+1),v(:,k+1),omega(:,k+1),quat(:,k+1)] = odometryUAV(odomSub);
    
    euler_p(:,k+1) = Euler_p(omega(:,k+1),euler(:,k+1));
      
    R(:,:,k) = quatToRot(quat(:,k+1));

    u(:,k+1) = inv(R(:,:,k))*v(:,k+1);
                     
    while toc<ts
    end
    
    % 3) Tiempo de mÃ¡quina   
     dt(k) = toc;
 
end
%%

u_ref = [0.0, 0, 0, 0];
send_velocities(velControl_topic, velControl_msg, u_ref);

s0 = "IdentificacionAlgoritmos/";
s2 = '.mat';

% Pregunta al usuario si desea guardar como simulación o en la vida real
choice = input('¿Desea guardar como simulación o en la vida real? (sim/real): ', 's');

if strcmpi(choice, 'sim')
    % Sección para guardar como simulación   
    s1 = 'M100_ident_sim_';
    s = strcat(s0,s1,n_chr,s2)
    save(s,"h","v","euler","euler_p","u","quat","ts","omega","t","v_ref","R")
    disp('Guardando como simulación...');
    
elseif strcmpi(choice, 'real')
    % Sección para guardar en la vida real
    s1 = 'M100_ident_real_';
    s = strcat(s0,s1,n_chr,s2)
    disp('Guardando en la vida real...');
    
else
    disp('Opción no reconocida. Saliendo...');
end



%%
close all

subplot(4,1,1)
plot(v_ref(1,:));hold on
plot(u(1,:));hold on
plot(v_p(1,:));
legend("ul_r","ul","ulp");

subplot(4,1,2)
plot(v_ref(2,:));hold on
plot(u(2,:));hold on
plot(v_p(2,:));
legend("um","um","ump");

subplot(4,1,3)
plot(v_ref(3,:));hold on
plot(u(3,:));hold on
plot(v_p(3,:));
legend("um","um","ump");

subplot(4,1,4)
plot(v_ref(4,:)); hold on
plot(omega(3,:))
legend("w");

%%
% figure(2)
% subplot(4,1,1)
% plot(x(1,:))
% subplot(4,1,2)
% plot(x(2,:))
% subplot(4,1,3)
% plot(x(3,:))

%%
figure(3)
subplot(4,1,1)
plot(euler(1,:))
xlabel("Time");
legend("phi-ROLL");
subplot(4,1,2)
plot(euler(2,:))
legend("theta-PITCH");
subplot(4,1,3)
plot(euler(3,:))
legend("psi-YAW");

%%
figure(4)
subplot(4,1,1)
plot(euler_p(1,:))
xlabel("Time");
legend("phi-ROLL_p");
subplot(4,1,2)
plot(euler_p(2,:))
legend("theta-PITCH_p");
subplot(4,1,3)
plot(euler_p(3,:))
legend("psi-YAW_p");

%% SAVE DATA
% save("Test1.mat");

filename = 'Test5.mat';
save(filename)