function [hd] = Velocities_data(n,t,w_ref_active,banda )


%% VELOCIDADES PARA EL UAV
switch n    
    case 1
        % velocidades escalonadas 
        close all
        b=0;
        R=rand(1,6);
        %banda=20;
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

            ul_ref(k)=0.0*V(k,1);
            um_ref(k)=-0.5*V(k,2);
            un_ref(k)=0.5*V(k,3);
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
        ul_ref=0.1*sin(1*t).*cos(0.1*t)-0.15*cos(1*t);
        um_ref=0.1*cos(0.5*t)-0.15*sin(0.75*t);       
        un_ref=0.1*sin(0.1*t).*cos(-0.5*t)-0.15*cos(0.1*t); 
        w_ref=0.1*cos(0.5*t)+0.3*sin(0.5*t);
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
        ul_ref=1.9*sin(0.5*t).*cos(0.5*t)+0.15*cos(0.5*t);
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
        %%
        close all
        %velocidades de referncia 1
        ul_ref=0.4*cos(1*t)+0.3*sin(0.4*t);
        um_ref=0.2*cos(t).*cos(0.5*t)+0.25*sin(0.7*t);
        un_ref=0.5*sin(t).*cos(0.5*t);
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
        ul_ref=0.5*(0.7*cos(1*t)+0.5*sin(3*t));
        
        um_ref=0.6*(0.7*cos(2*t)+0.2*sin(0.4*t));
        un_ref=0.6*(0.25 *cos(3*t)+ 0.6 *sin(1*t)); 
        w_ref=1*(0.25 *cos(2*t)+ 0.25 *sin(0.4*t));
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);   
        %%
    case 12
        close all
        
        ul_ref=0.7*cos(0.5*t)+0.2*sin(0.4*t);
        um_ref=0.1*cos(1*t)+0.1*cos(2*t);
        un_ref=0.1*cos(2*t)+0.1*cos(5*t);
        w_ref=1*sin(2*t);
                
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref); 
        %%
     case 13
        close all
        
        ul_ref=0.1*cos(1*t)-0.1*cos(2*t);
        um_ref=0.1*cos(1*t)+0.1*cos(2*t);
        un_ref=0.1*cos(2*t)+0.1*cos(5*t);
        w_ref=0.2*sin(2*t);
                
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
    case 14
        close all
        
        ul_ref=0.4*cos(1*t)-0.2*cos(3*t);
        um_ref=0.4*cos(3*t)+0.2*cos(1*t);
        un_ref=0.4*cos(2*t)-0.2*cos(5*t);
        w_ref=0.2*sin(2*t);
                
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
        %%
    case 15
        close all
        
        ul_ref=0.4*cos(1*t)-0.2*cos(3*t);
        um_ref=0.4*cos(3*t)+0.2*cos(1*t);
        un_ref=0.4*cos(2*t)-0.2*cos(5*t);
        w_ref=1.99*sin(2*t).*cos(0.5*t);
                
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
        
    case 16
       close all
        pulses = zeros(size(t));   % Initialize pulse vector

    % Define pulse times and magnitudes
    pulse_info = [
        1.5, -0.2;   % Time = 1.5 seconds, Magnitude = -0.2
        3.2, 0;      % Time = 3.2 seconds, Magnitude = 0 (Amplitude zero)
        5.0, 0.2;    % Time = 5.0 seconds, Magnitude = 0.2
        7.0, 0.5;    % Time = 7.0 seconds, Magnitude = 0.1
        10.0, -0.2;  % Time = 10.0 seconds, Magnitude = -0.2
        14.0, 0;     % Time = 14.0 seconds, Magnitude = 0 (Amplitude zero)
        15.5, -0.3;  % Time = 15.5 seconds, Magnitude = -0.3
        18.0, 0.3;   % Time = 18.0 seconds, Magnitude = 0.3
        19.7, 0;     % Time = 19.7 seconds, Magnitude = 0 (Amplitude zero)
        21.0, -0.1;  % Time = 21.0 seconds, Magnitude = -0.1
        22.5, 0.4;   % Time = 22.5 seconds, Magnitude = 0.4
    ];

    for i = 1:size(pulse_info, 1)
        pulse_time = pulse_info(i, 1);
        pulse_magnitude = pulse_info(i, 2);
        
        % Set the pulse magnitude at the specified time
        pulse_index = find(t >= pulse_time, 1);
        pulses(pulse_index:end) = pulse_magnitude;
    end

    ul_ref = zeros(size(t));
    um_ref = zeros(size(t));
    un_ref = zeros(size(t));
    w_ref = zeros(size(t));

    for k = 1:length(t)
        ul_ref(k) = 0.0 * pulses(k);
        um_ref(k) = -0.5 * pulses(k);
        un_ref(k) = 0.5 * pulses(k);
        w_ref(k) = 0.75 * pulses(k);
    end

    subplot(4, 1, 1)
    plot(ul_ref);
    title('ul\_ref');
    subplot(4, 1, 2)
    plot(um_ref);
    title('um\_ref');
    subplot(4, 1, 3)
    plot(un_ref);
    title('un\_ref');
    subplot(4, 1, 4)
    plot(w_ref);
    title('w\_ref');
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

hd = [ul_ref;um_ref;un_ref;w_ref];

end

