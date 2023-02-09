function [state] = State_predict( pre_state, sensors_info)

% sensors_working -(1) IMU
%                  (2) GPS
%                  (3) Velocimetro
%                  (4) encoder
%                  (5) potenciometer
%                  (6) tachometer
% if one then sensor working
% if zero then sensor not working


%state(1).x = [x;y;vx;vy]
%state(1).P = init value Qn (f*eye(4))

%state(2).x = [teta;w]
%state(2).P = init value Qn (f*eye(2))

%state(3).x = [phi_1;phi_2]
%state(3).P = init value Qn (f*eye(2))


% kalman filter para estimar o estado do carro
    global h sensors_working stop_car
    L=2.2;
    %estimar a posição(x,y)
        if sum(sensors_working(1:3)) == 0
            fprintf("\n Stop The Car!  Impossibel to detect Car movement!");
            stop_car = 1;
        end
            y = [sensors_info.x, sensors_info.y, sensors_info.vx,sensors_info.vy]';
            u = [sensors_info.ax,sensors_info.ay]';
            [state(1).x,state(1).P] = pos_tracker(pre_state(1).x,y,u,h,pre_state(1).P);
   
    %estimar phi
    
        y = [sensors_info.phi_1, sensors_info.phi_2, 0]';
        u = [sensors_info.ws_1, sensors_info.ws_2]';
        [state(3).x,state(3).P,state(3).Y] = phi_tracker(pre_state(3).x,y,u,h,pre_state(3).P);
        state(3).ws = mean(u);
        
     %estimar teta
        if sensors_working(1) == 1           
            y = [sensors_info.teta, sensors_info.w]';
            u = [sensors_info.aw]';
            [state(2).x,state(2).P] = teta_tracker(pre_state(2).x,y,u,h,pre_state(2).P);
        else
            y = [sensors_info.teta, sensors_info.w]';
            u = [tan(state(3).Y)/L* norm([state(1).x(3), state(1).x(4)])]';
            [state(2).x,state(2).P] = teta_tracker_IMU_FAIL(pre_state(2).x,y,u,h,pre_state(2).P);    
        end
 
end

%estimação de dados atraves de kalman filters

function [x,P] = pos_tracker(x,y,u,h,P)
        global sensors_working sensors_state_change
   
    if (sensors_working(1) ==0)
        u=[0 0]';
    end
    %model
    A = [1 0 h 0;
         0 1 0 h;
         0 0 1 0;
         0 0 0 1];
     
    B = [(h.^2)/2      0   ;
             0     (h.^2)/2;
             h         0   ;
             0         h   ];
         
    C = eye(4);
    %austar estes com o sensors_working
    Qn = 0.1*eye(4);
   
    Rn = 0.01*eye(4);
    Rn(1,1) = 1.1;
    Rn(2,2) = 1.1;
    if not(sensors_working(2)) && sensors_working(3)
        C =[0 0 1 0;
            0 0 0 1];
        
        aux = y(3:4);
        y =[];
        y = aux;
        Rn=[];
        Rn = 0.01*eye(2);
        if(sensors_state_change(2))
            P = Qn;
            sensors_state_change(2) = 0;
        end

    end
    if not(sensors_working(3)) && sensors_working(2)
        C =[1 0 0 0;
            0 1 0 0];
        
        aux = y(1:2);
        y =[];
        y = aux;
        Rn=[];
        Rn = 0.01*eye(2);
        Rn(1,1) = 1.1;
        Rn(2,2) = 1.1;
        if(sensors_state_change(3))
            P = Qn;
            sensors_state_change(3) = 0;
        end

    end
    
    
    
    %prediction
    x = A*x + B*u; 
    
    
    if (sensors_working(2) || sensors_working(3))
        %update
        P = A*P*A'+Qn;
        S = C*P*C'+Rn;
        K = P*C'/S;
        x = x + K*( y - C*x);
    end
end

function [x,P] = teta_tracker(x,y,u,h,P)
    
    global sensors_state_change
    
    %model
    A = [1 h;
         0 1];
    B = [(h.^2)/2  h]';
    C = eye(2);
    Qn = 0.1*eye(2);
    Rn = 0.01*eye(2);
    
    if(sensors_state_change(1))
        P = Qn;
        sensors_state_change(1) = 0;
    end
    
    %prediction
    x = A*x + B*u; 
    P = A*P*A'+Qn;

    %update
    S = C*P*C'+Rn;
    K = P*C'/S;
    x = x + K*( y - C*x);
end

function [x,P] = teta_tracker_IMU_FAIL(x,y,u,h,P)
    
    global sensors_state_change
    
    %model
    A = [1];
    B = [h]';

    Qn = 0.1;
    
    if(sensors_state_change(1))
        P = Qn;
        sensors_state_change(1) = 0;
    end
    
    %prediction
    x = A*x + B*u; 


end

function [x,P,Y] = phi_tracker(x,y,u,h,P)  

    global sensors_working stop_car sensors_state_change

    update  = 1;
    prediction = 1;
    
    
    if( sensors_working(4) && sensors_working(5) && sensors_working(6) )
        if 1 == length(x)
            x=[x x]';
        end
        %model
        A = [1 0;
             0 1];
        B = [h 0;
             0 h];
        C = [0.5 0.5;
             0.5 0.5;
              1  -1];            
        Qn = 0.1*eye(2);
        Rn = 0.01*eye(3);

    elseif not(sensors_working(4)) && sensors_working(5) && sensors_working(6)
        
        aux=u(2);
        u=NaN;
        u=aux;
        
        aux=y(2);
        y=NaN;
        y=aux;
        
        aux = mean(x);
        x=NaN;
        x=aux;
        
        A = 1;
        B = h;
        C = 1;
        
        Qn = 0.1;
        Rn = 0.1;
        
    elseif sensors_working(4) && not(sensors_working(5)) && sensors_working(6)
        
        if 1 == length(x)
            x=[x x]';
        end
        
        aux=y(1);
        y=NaN;
        y=aux;
        
        A = [1 0;
             0 1];
        B = [h 0;
             0 h];
        C = [0.5 0.5];

        Qn = 0.1*eye(2);
        Rn = 0.1;
        
    elseif sensors_working(4) && sensors_working(5) && not(sensors_working(6))
        
        if 1 == length(x)
            x=[x x]';
        end
        
        u(2)=u(1);
        
        %model
        A = [1 0;
             0 1];
        B = [h 0;
             0 h];
        C = [0.5 0.5;
             0.5 0.5;
              1  -1];

        Qn = 0.1*eye(2);
        Rn = 0.1*eye(3);
    elseif sensors_working(4) && not(sensors_working(5)) && not(sensors_working(6))
        aux=u(1);
        u=NaN;
        u=aux;
        
        aux=y(1);
        y=NaN;
        y=aux;
        
        aux = mean(x);
        x=NaN;
        x=aux;
        
        A = 1;
        B = h;
        C = 1;
        
        Qn = 0.1;
        Rn = 0.1;
        
    elseif not(sensors_working(4)) && not(sensors_working(5)) && sensors_working(6)
        
        aux=u(2);
        u=NaN;
        u=aux;
        
        aux = mean(x);
        x=NaN;
        x=aux;
        
        A = 1;
        B = h;
        C = 1;
        
        Qn = 0.1;
        Rn = 0.1;
        
        update = 0;
    
    elseif not(sensors_working(4)) && sensors_working(5) && not(sensors_working(6))
        
        Y = y(2);
        x = y(2);
        
        Qn = 0.1;
        
        update = 0;
        prediction = 0;
        
        
    elseif  not(sensors_working(4)) && not(sensors_working(5)) && not(sensors_working(6))
        update = 0;
        prediction = 0;
        
        Y = mean(x);
        Qn = 0.1;
        fprintf("\nStop The Car!  Steering Wheel sensors are dead!");
        stop_car = 1;
    end
    
    if( sensors_state_change(4) || sensors_state_change(5)||sensors_state_change(6))
        P = Qn;
        sensors_state_change(4) = 0;
        sensors_state_change(5) = 0;
        sensors_state_change(6) = 0;
    end
    
    if prediction
        %prediction
        x = A*x + B*u; 
        P = A*P*A'+Qn;
    end
    
    if(update)
        %update
        S = C*P*C'+Rn;
        K = P*C'/S;
        x = x + K*( y - C*x);
        if length(x) == 1
            Y = x;
        else
            Y = [0.5 0.5] * x;
        end
    end
    if not(update) && prediction
        if length(x) == 1
            Y = x;
        else
            Y = [0.5 0.5] * x;
        end
    end
end



