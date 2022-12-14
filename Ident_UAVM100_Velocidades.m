%% Inicializacion
tic
clc; clear all; close all; warning off % Inicializacion
ts = 1/30;       
tfin = 45;     
t = 0:ts:tfin;

%% Inicializa Nodo ROS
rosshutdown
setenv('ROS_MASTER_URI','http://192.168.88.251:11311')
setenv('ROS_IP','192.168.88.247')
rosinit

%% CONSTANTS VALUES OF THE ROBOT
a = 0.0; 
b = 0.0;
c = 0.0;
L = [a, b, c];

%% 1) PUBLISHER TOPICS & MSG ROS - UAV M100

[velControl_topic,velControl_msg] = rospublisher('/m100/velocityControl','geometry_msgs/Twist');
u_ref = [0, 0, 0, 0.0];
send_velocities(velControl_topic, velControl_msg, u_ref);

%% 2) Suscriber TOPICS & MSG ROS - UAV M100

odomSub = rossubscriber('/dji_sdk/odometry');

    [h(:,1),hp(:,1)] = odometryUAV(odomSub);
    xu(1) = h(1,1);
    yu(1) = h(2,1);
    zu(1) = h(3,1) ;   
    psi(1) = Angulo(h(4,1));
      
    ul(1) = hp(1,1);
    um(1) = hp(2,1);
    un(1) = hp(3,1);
    w(1) = hp(4,1);

%% 
while 1

continuar=input('PULSA CUALQUIER TECLA PARA CONTINUAR');
break
end 

w_ref_active = true;
n=2; % Seleccion UAV

%% VELOCIDADES PARA EL UAV
switch n    
    case 1
        % velocidades escalonadas 
        close all
        b=0;
        R=rand(1,6);
        banda=80;
        for k=1:length(t)
    
            if b<=banda
            a(k)=-0.5;
            b=b+1;
                aux=1;
            end
            if b>banda && b<=2*banda
            a(k)=1;
            aux=2;
            b=b+1;
            end   

            if aux==1
            V(k,:)=abs(R);
            end
            if aux==2
            V(k,:)=-abs(R);
            end
    
            if b>2*banda
            b=0;
            R=rand(1,6);
            end

            ul_ref(k)=0.7*V(k,1);
            um_ref(k)=-0.7*V(k,2);
            un_ref(k)=0.7*V(k,3);
            %w_ref(k)=0.75*V(k,4);

        end
        
        banda_=125;
        for k=1:length(t)
    
            if b<=banda_
            a(k)=-1;
            b=b+1;
                aux=1;
            end
            if b>banda_ && b<=2*banda_
            a(k)=1;
            aux=2;
            b=b+1;
            end   

            if aux==1
            V(k,:)=abs(R);
            end
            if aux==2
            V(k,:)=-abs(R);
            end
    
            if b>2*banda_
            b=0;
            R=rand(1,6);
            end

            %ul_ref(k)=1.25*V(k,1);
            %um_ref(k)=-1.25*V(k,2);
            %un_ref(k)=1.0*V(k,3);
            w_ref(k)=0.75*V(k,4);

        end
            
        subplot(4,1,1)
        plot(t,ul_ref);
        subplot(4,1,2)
        plot(t,um_ref);
        subplot(4,1,3)
        plot(t,un_ref);
        subplot(4,1,4)
        plot(t,w_ref); 
        %%
    case 2
        %velocidades de referncia 1
        close all 
        ul_ref=0.5*sin(1*t).*cos(0.1*t)-0.15*cos(1*t);
        um_ref=0.5*cos(0.5*t)-0.15*sin(0.75*t);       
        un_ref=0.4*sin(0.1*t).*cos(-0.5*t)-0.15*cos(0.1*t); 
        w_ref=0.4*cos(0.5*t)+0.3*sin(0.5*t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);       
        %%
    case 3
        % velocidades de referncia 2
        close all
        ul_ref=0.9*sin(0.5*t).*cos(0.5*t)+0.15*cos(0.5*t);
        um_ref=0.4*cos(t)+0.3*cos(0.5*t);
        un_ref=0.1*sin(0.6*t).*sin(0.5*t)+0.3*cos(0.7*t).*cos(0.4*t);
        w_ref=0.3*sin(0.2*t).*cos(0.4*t)-0.15*cos(t); 
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
        %%
    case 4
        % % velocidades de referncia 3
        close all
        ul_ref=0.5*sin(1*t).*cos(0.1*t)-0.15*cos(1*t);
        um_ref=0.4*cos(t)+0.3*cos(0.9*t);
        un_ref=0.5*sin(0.75*t).*cos(0.1*t)-0.15*cos(t);
        w_ref=0.25 *cos(0.5*t)+ 0.25 *sin(0.4*t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref); 
          
        %%
    case 5
        %  velocidades de referncia 5
        close all
        ul_ref=0.8*sin(0.1*t).*cos(t)+0.15*cos(0.1*t);
        um_ref=0.8*sin(0.5*t).*cos(0.4*t)-0.15*cos(t);
        un_ref=0.4*sin(0.1*t).*cos(0.5*t)-0.15*cos(0.1*t); 
        w_ref=0.8*sin(0.25*t).*cos(0.4*t)-0.15*cos(t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
    %%
    case 6
        %  velocidades de referncia 5
        close all
        ul_ref=0.4*sin(0.1*t).*cos(0.5*t)-0.15*cos(0.1*t);
        um_ref=0.4*cos(0.1*t)+0.3*cos(0.4*t);
        un_ref=0.1*sin(0.6*t).*sin(0.5*t)+0.3*cos(0.7*t).*cos(0.4*t);
        w_ref=0.25 *cos(0.5*t)+ 0.25 *sin(0.4*t); 
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
    %%
    case 7
        close all
        % velocidades de referncia 2
        ul_ref=0.2*sin(0.75*t).*cos(0.8*t)-0.15*cos(t);
        um_ref=0.4*cos(0.1*t)+0.3*cos(0.4*t);
        un_ref=0.5*sin(t).*sin(0.5*t)+0.3*cos(0.7*t).*cos(0.3*t);
        w_ref=0.7*cos(0.2*t)+0.2*sin(0.4*t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
        %%
    case 8
        close all
        % % velocidades de referncia 3
        ul_ref=0.2*sin(0.75*t).*cos(0.8*t)-0.15*cos(t);
        um_ref=0.4*cos(0.1*t)+0.3*cos(0.4*t);
        un_ref=0.2*sin(0.5*t).*sin(0.5*t)+0.25*cos(0.4*t).*cos(0.7*t);
        w_ref=cos(0.2*t)+sin(0.4*t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
        %%
    case 9
        close all
        % velocidades de referncia 4
        ul_ref=0.4*sin(0.1*t).*cos(0.5*t)-0.15*cos(0.1*t);
        um_ref=0.2*cos(t)+0.3*cos(0.5*t);
        un_ref=0.1*sin(0.6*t).*sin(0.5*t)+0.3*cos(0.7*t).*cos(0.4*t);
        w_ref=0.8*sin(0.5*t).*cos(0.4*t)-0.15*cos(t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
        %%
        
    case 10
        close all
        %velocidades de referncia 1
        ul_ref=0.5*sin(t).*cos(0.5*t);
        um_ref=0.2*cos(t).*cos(0.5*t)+0.25*sin(0.7*t);
        un_ref=0.4*cos(0.1*t)+0.3*sin(0.4*t);
        w_ref=0.5*sin(t).*sin(0.5*t)+0.3*cos(0.7*t).*cos(0.3*t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
        
     case 11
        close all
        %velocidades de referncia 1
        ul_ref=0.5*(0.25 *cos(0.5*t)+ 0.25 *sin(0.4*t));
        um_ref=0.5*(0.7*cos(0.2*t)+0.2*sin(0.4*t));
        un_ref=0.5*(0.25 *cos(0.5*t)+ 0.25 *sin(0.4*t)); 
        w_ref=0.5*(0.7*cos(0.2*t)+0.2*sin(0.4*t));
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);   
        %%
    
        
        %%
    otherwise
        disp("Ninguno de los anteriores");
end
%      

if w_ref_active
    disp("w_ref != 0")   
else  
    w_ref= 0*t;
    disp("w_ref = 0")
end


%% Matriz de rotacion


 
%% Comienza el programa
for k=1:length(t)
    tic
    
%% SEND VALUES OF CONTROL ROBOT  - UAV M100

    u_ref = [ul_ref(k), um_ref(k), un_ref(k),  w_ref(k)];
    send_velocities(velControl_topic, velControl_msg, u_ref);
    
    %% 3) Odometria del UAV
    try
    [h(:,k),hp(:,k)] = odometryUAV(odomSub);
    xu(k+1) = h(1,k);
    yu(k+1) = h(2,k);
    zu(k+1) = h(3,k) ;   
    psi(k+1) = Angulo(h(4,k));
      
    R = [cos(psi(k)) -sin(psi(k)) 0 0;
    sin(psi(k)) cos(psi(k)) 0 0;
    0 0 1 0
    0 0 0 1];

    
    u(:,k) = inv(R)*hp(:,k);
    
    
    ul(k) = u(1,k);
    um(k) = u(2,k);
    un(k) = u(3,k);
    w(k) = u(4,k);
         
    catch
          
    xu(k+1) = xu(k);
    yu(k+1) = yu(k);
    zu(k+1) = zu(k) ;   
    psi(k+1) = psi(k);
     
    ul(k+1) = ul(k);
    um(k+1) = um(k);
    un(k+1) = un(k);
    w(k+1) = w(k);
    end
    

    
    while toc<ts
    end
    
% 3) Tiempo de mÃ¡quina   
     dt(k) = toc;
     dt2(k) = toc;

end
%%

u_ref = [0, 0, 0, 0];
send_velocities(velControl_topic, velControl_msg, u_ref);

%%

plot(um_ref)
hold on
plot(um)
close all

subplot(4,1,1)
plot(ul_ref);hold on
plot(ul)
subplot(4,1,2)
plot(um_ref);hold on
plot(um)
subplot(4,1,3)
plot(un_ref);hold on
plot(un)
subplot(4,1,4)
plot(w_ref); hold on
plot(w)

%% SAVE DATA
% save("Test1.mat");

filename = 'Test5.mat';
save(filename)
