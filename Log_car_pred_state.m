function [car_pred_state_log] = Log_car_pred_state(car_pred_state_log,New_state)
        %save new state
        car_pred_state_log(end+1).x = New_state.x;
        car_pred_state_log(end).y = New_state.y;
        car_pred_state_log(end).phi = New_state.phi;
        car_pred_state_log(end).teta = New_state.teta;
        car_pred_state_log(end).V = New_state.V;
        car_pred_state_log(end).Ws = New_state.Ws;
        
end

