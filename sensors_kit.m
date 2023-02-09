
% função que reune os dados de todos os sensores
function [sensor_info] = sensors_kit(state)

    %sensores
    [IMU_info] = IMU(state);
    
    [GPS_info] = GPS(state);
    
    [Velocimetro_info] = Velocimetro(state);
    
    [encoder_info] = encoder(state);
    
    [potenciometer_info] = potenciometer(state);
    
    [tachometer_info] = tachometer(state);
    
    %IMU
    sensor_info.ax = IMU_info.ax;
    sensor_info.ay = IMU_info.ay;
    sensor_info.aw = IMU_info.aw;
    sensor_info.w = IMU_info.w;
    sensor_info.teta = IMU_info.teta;
    
    %GPS
    sensor_info.x = GPS_info.x;
    sensor_info.y = GPS_info.y;
    
    %Velocimetro
    
    sensor_info.vx = Velocimetro_info.vx;
    sensor_info.vy = Velocimetro_info.vy;
    
    %encoder
    sensor_info.phi_1 = encoder_info.phi_1;
    sensor_info.ws_1 = encoder_info.ws_1;
    
    %potenciometer
    sensor_info.phi_2 = potenciometer_info.phi_2;
    
    %tachometer
    sensor_info.ws_2 = tachometer_info.ws_2;
end

% funções que representam a aquisição de informação dos sensores

function [sensor_info] = IMU(state_log)
    global h
    L=2.2;
    sensor_info.ax =((state_log(end).V*cos(state_log(end).teta + state_log(end).phi))-(state_log(end-1).V*cos(state_log(end-1).teta + state_log(end-1).phi )))/h + 0.05*randn;
    sensor_info.ay =((state_log(end).V*sin(state_log(end).teta + state_log(end).phi))-(state_log(end-1).V*sin(state_log(end-1).teta + state_log(end-1).phi)))/h + 0.05*randn;
    sensor_info.aw =(state_log(end).V .* tan(state_log(end).phi)-state_log(end-1).V .* tan(state_log(end-1).phi))/(h*L) + 0.005*randn;
    sensor_info.w =(state_log(end).V .* tan(state_log(end).phi))/L + 0.01*randn;
    sensor_info.teta =state_log(end).teta + 0.01*randn;
    
end

function [sensor_info] = GPS(state_log)

    sensor_info.x = state_log(end).x + 0.05*randn;
    sensor_info.y = state_log(end).y + 0.05*randn;
      
end

function [sensor_info] = Velocimetro(state_log)

    sensor_info.vx = state_log(end).V*cos(state_log(end).teta + state_log(end).phi) + (state_log(end).V)*0.05*randn;
    sensor_info.vy = state_log(end).V*sin(state_log(end).teta + state_log(end).phi) + (state_log(end).V)*0.05*randn;
end

function [sensor_info] = encoder(state_log)
    
    sensor_info.phi_1 = state_log(end).phi + 0.01*randn;
    sensor_info.ws_1 = state_log(end).Ws + 0.01*randn;
    
end

function [sensor_info] = potenciometer(state_log)
    
    sensor_info.phi_2 = state_log(end).phi + 0.01*randn;
    
end

function [sensor_info] = tachometer(state_log)
    
    sensor_info.ws_2 = state_log(end).Ws + 0.01*randn;
 
end


