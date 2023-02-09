function [Navigation_State, real_car_state_log, car_pred_state_log] = init_state(trajectory)
    
    pos = [trajectory(1,1) , trajectory(2,1)]+ 0.1*rand(1,2);
    Theta = atan2(trajectory(2,2)-trajectory(2,1), trajectory(1,2)-trajectory(1,1));
    Theta_N = Theta + 2/180*pi*rand();
    
    %nav init state
    Navigation_State(1).x = [pos(1); pos(2); 0; 0];
    Navigation_State(1).P = 0.01*eye(4);
    
    Navigation_State(2).x = [Theta_N;0];
    Navigation_State(2).P = 0.01*eye(2);
    
    Navigation_State(3).x = [0;0];
    Navigation_State(3).P = 0.01*eye(2);
      
    %car real state init
    real_car_state_log(1).x = trajectory(1,1);
    real_car_state_log(1).y = trajectory(2,1);
    real_car_state_log(1).teta = Theta;
    real_car_state_log(1).phi = 0;
    real_car_state_log(1).V = 0;
    real_car_state_log(1).Ws = 0;
    
    %car pred state init
    car_pred_state_log(1).x = pos(1);
    car_pred_state_log(1).y = pos(2);
    car_pred_state_log(1).teta = Theta_N;
    car_pred_state_log(1).phi = 0;
    car_pred_state_log(1).V = 0;
    car_pred_state_log(1).Ws = 0;
end

