function [state_log] = Car(V, Ws, state_log)

global h

last_pos = length(state_log);

%realistic Ws

offset = 3*pi/180;
noise_ws = 1*pi/180 * Ws/pi * randn;
Ws_r = Ws + offset + noise_ws;
state_log(last_pos+1).Ws = Ws;
%realistic V

noise_v = 0.01 * V * randn;
V_r = V + noise_v;
state_log(last_pos+1).V = V;

% Car simulation
[state_log(last_pos+1).x, state_log(last_pos+1).y, state_log(last_pos+1).teta, state_log(last_pos+1).phi] = car_model(h,V_r,Ws_r,state_log(last_pos));

end

function [x, y, teta, phi] = car_model(h,V,Ws,last_full_state)
%car_sim determina o estado atual do carro partindo do estado passado,
%usando a regra do trapesio


% last_full_state - contem as variaveis V, Ws, x, y, teta, phi do instante
% anterior

%medidas do carro
L=2.2;

% determinar o phi atual
phi = last_full_state.phi + h/2 .* (Ws + last_full_state.Ws);

% determinar o teta atual
teta  = last_full_state.teta + h/2 .* ( (V.*tan(phi)/L) + (last_full_state.V .* tan(last_full_state.phi)/L) );

% determinar o y atual
y  = last_full_state.y + h/2 .* ( ( V.*sin(teta) ) + ( last_full_state.V .* sin(last_full_state.teta) ) );

% determinar o y atual
x  = last_full_state.x + h/2 .* ( ( V.*cos(teta) ) + ( last_full_state.V .* cos(last_full_state.teta) ) );



end
