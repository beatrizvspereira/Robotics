function [V,Ws,n_pe,E_spent,E_remaining] = control(final_pos,x,y,theta,phi, last_V,next_full_state, E_spent, E_bg, E_remaining, N_steps, Po, v_max)
%CONTROL - Função utilizada para controlo do carro
% Inputs: 
% final_pos - posição final que queremos que o carro esteja
% x,y - posição atual do carro 
% theta - orientação atual do carro
% phi - orientação atual do volante do carro
% last_V - velocidade anterior do carro
% next_full_state - estrutura com informação sobre a próxima posição do
% carro
% E_spent - Energia gasta até ao momento
% E_bg - Energia total disponível para o caminho
% E_remaining - Energia restante para fazer o caminho
% N_steps - Aproximação do número de passos restantes para terminar a
% trajectória
% Po - Energia que o carro gasta quando está parado mas ligado
% v_max - velocidade máxima que o carro pode atingir
%
% Outputs:
% V - velocidade linear do carro
% Ws - velocidade angular do carro
% n_pe - erro de posição relativamente ao ponto intermédio no caminho
% E_spent - Energia gasta neste passo
% E_remaining - Energia restante depois deste passo

global h

M=810; %kg

final_post_error=[x y]-final_pos;
post_error=norm(final_post_error);

post_e=[x;y]-[next_full_state.x;next_full_state.y];
n_pe=norm(post_e);

%Orientação 'ideal'
beta=atan2(-post_e(2),-post_e(1));

%Erro de orientação
alpha=beta-(theta+phi);

if alpha > pi
        alpha = alpha - 2*pi;
elseif alpha < -pi
        alpha = alpha + 2*pi;
end

%Energia disponível por passo
E_step=E_remaining/N_steps;
%Velocidade máxima para atingirmos o fim da trajectória
v_max_1=E_step/(Po*h);

if v_max_1>(40/3.6) || v_max_1<0
  v_max_1=(40/3.6);
end

%Velocidade angular máxima
v_max_2=(pi/4);
%Acelaração máxima
a_max = 10*h;

K1=0.02*exp(-abs(90*alpha));
K2=1; K3=10;

V=v_max_1*tanh(K1*post_error);

% Restrição da acelaração
if (a_max < abs((V-last_V)/h))
    V = last_V + sign(V-last_V)*a_max*h;
end

% Velocidade máxima defenida por um sinal (evento E2)
if V>v_max
    V=v_max;  
end

Ws=v_max_2*tanh(((1+(K2*(beta/alpha)))*(tanh(K1*post_error)/post_error)*sin(alpha)+K3*tanh(alpha)));

dv=(V-last_V)/h;
dE=(M*abs(dv)*abs(V)+Po)*h;

%Energia gasta até ao momento
E_spent=E_spent+dE;
%Energia restante
E_remaining=E_bg-E_spent;


end

