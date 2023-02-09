function mainFunction(app)
    
    %Initialize the displays
    app.EnergyGauge.Value = 100;
    app.VelocityGauge.Value = 0;
    
    %Upload Imagem Mapa
    map = imread('MapBinFinal.bmp');
    map = imcomplement(map);

    trajectory = trajectoryGen(app);
    %%If the chosen points are invalid
    if(trajectory == -1)
        return;
    end
    

%defenir estado (navegação + real + estimado)
    global sensors_working sensors_state_change
    sensors_working = ones(1,6);
    sensors_state_change = zeros(1,6);
    global h stop_car
    h = 0.1;
    stop_car=0;
    disable =zeros(1,6);
    [Navigation_State, real_car_state_log, car_pred_state_log] = init_state(trajectory);
    
    [path_x,path_y,f1,f2,f3,f4,t_stop,E_spent,E_bg,j,k,p,E_remaining,Po,N_steps,v_max,f4_act,final_post,car_control,sorted_flags] = init_control(trajectory);
    
%loop while(not(end_of_route))
for i=1:length(path_x)
    next_full_state.x=path_x(i);
    next_full_state.y=path_y(i);
    post_e=[car_pred_state_log(end).x;car_pred_state_log(end).y]-[next_full_state.x;next_full_state.y];
    n_pe=norm(post_e);
    
    %Lidar com evento 1,2 e 3
    [final_post,v_max,f1,f2,f3,k,sorted_flags] = handle_events(path_x,path_y,trajectory,i,f1,f2,f3,final_post,v_max,k,sorted_flags);
    
    if next_full_state.x==path_x(end) && next_full_state.y==path_y(end)
        threshold=0.5;
    else
        threshold=2.5;
    end
   
    %loop while(not(close))
    while n_pe>threshold && E_remaining>0 && ~stop_car
        if(app.StartSwitch.Value == "Off")
            stop_car = 1;
        end
        
        %Ativar sensor fault
        %IMU Fault
        if (app.IMUFaultSwitch.Value == "On")
            disable(1) = 1;
        else
            disable(1) = 0;
        end
        %GPS Fault
        if (app.GPSFaultSwitch.Value == "On")
            disable(2) = 1;
        else
            disable(2) = 0;
        end
           
        %Speedometer Fault
        if (app.SpeedometerFaultSwitch.Value == "On")
            disable(3) = 1;
        else
            disable(3) = 0;
        end
        %Encoder Fault
        if (app.EncoderFaultSwitch.Value == "On")
            disable(4) = 1;
        else
            disable(4) = 0;
        end
        
        %Potenciometer Fault
        if (app.PotenciometerFaultSwitch.Value == "On")
            disable(5) = 1;
        else
            disable(5) = 0;
        end 
        
        %Tacometer Fault
        if (app.TacometerFaultSwitch.Value == "On")
            disable(6) = 1;
        else
            disable(6) = 0;
        end
        sensors_breakdown(disable);

        
        %Bloco de controlo
        [V,Ws,n_pe,E_spent(j), E_remaining] = control(final_post,car_pred_state_log(end).x,car_pred_state_log(end).y,car_pred_state_log(end).teta, ...
                               car_pred_state_log(end).phi,car_pred_state_log(end).V, next_full_state, E_spent(j-1), E_bg, E_remaining, N_steps-(j-2), Po, v_max);
        
        %Parar o carro
        [final_post,V,Ws,k,p,f4,f4_act,sorted_flags] = stop_event(trajectory,i,j,k,p,path_x,path_y,...
                                                       f4_act,f1,f4,t_stop,final_post,V,Ws,sorted_flags);
        
        %Salva os controles
        [car_control] = Log_controls(V,Ws,car_control);
        
        %simula o funcionamento do carro real
        [real_car_state_log] = Car(V, Ws, real_car_state_log);

        %representa a aquisição de informação atraves de sensores 
        [sensors_info] = sensors_kit(real_car_state_log);

        %Determinar uma estimativa para o estado do carro
        [aux_state, Navigation_State] = Navigation(Navigation_State, sensors_info);
        
        %salva o estado
        [car_pred_state_log] = Log_car_pred_state(car_pred_state_log,aux_state);
        
        %Lidar com evento 4
        [final_post,f4_act] = handle_event4(V,i,j,f4,trajectory,[car_pred_state_log(end).x;car_pred_state_log(end).y],...
                              path_x,path_y,final_post,f4_act,sorted_flags);
        
        j=j+1;
      
    end
    app.EnergyGauge.Value = (E_remaining*100)/E_bg;
    app.VelocityGauge.Value = car_pred_state_log(end).V*3.6;
    
    %Detetar e assinalar colisão
    if length(real_car_state_log) == 1
        if(real_car_state_log(1).y>0 && real_car_state_log(1).x>0)&&(map(round(real_car_state_log(1).y), round(real_car_state_log(1).x)) == 0)
            app.ColisionLamp.Color = [1 0 0];
            stop_car=1;
        end
    else
        if(real_car_state_log(end).y>0 && real_car_state_log(end).x>0)&&(map(round(real_car_state_log(end).y), round(real_car_state_log(end).x)) == 0)
            app.ColisionLamp.Color = [1 0 0];
            stop_car=1;
        end
    end
    figure(1)
    hold on
    plot(real_car_state_log(end).x, real_car_state_log(end).y,'b.');
    pause(h*0.1)
    
    if(stop_car)   
        break;
    end
end
    

end