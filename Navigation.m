function [car_state, Nav_state] = Navigation( Nav_state, sensors_info)

%preve o estado 
[Nav_state] = State_predict( Nav_state, sensors_info);

%transporta os dados para o rejisto dos daos do carro
car_state.x = Nav_state(1).x(1);
car_state.y = Nav_state(1).x(2);
car_state.phi = Nav_state(3).Y;
car_state.teta = Nav_state(2).x(1);
car_state.V = norm([Nav_state(1).x(3),Nav_state(1).x(4)]);
car_state.Ws = Nav_state(3).ws;
end

