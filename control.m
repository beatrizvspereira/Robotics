function [V,Ws,n_pe,E_spent,E_remaining] = control(final_pos,x,y,theta,phi, last_V,next_full_state, E_spent, E_bg, E_remaining, N_steps, Po, v_max)
%CONTROL - Fun��o utilizada para controlo do carro
% Inputs: 
% final_pos - posi��o final que queremos que o carro esteja
% x,y - posi��o atual do carro 
% theta - orienta��o atual do carro
% phi - orienta��o atual do volante do carro
% last_V - velocidade anterior do carro
% next_full_state - estrutura com informa��o sobre a pr�xima posi��o do
% carro
% E_spent - Energia gasta at� ao momento
% E_bg - Energia total dispon�vel para o caminho
% E_remaining - Energia restante para fazer o caminho
% N_steps - Aproxima��o do n�mero de passos restantes para terminar a
% traject�ria
% Po - Energia que o carro gasta quando est� parado mas ligado
% v_max - velocidade m�xima que o carro pode atingir
%
% Outputs:
% V - velocidade linear do carro
% Ws - velocidade angular do carro
% n_pe - erro de posi��o relativamente ao ponto interm�dio no caminho
% E_spent - Energia gasta neste passo
% E_remaining - Energia restante depois deste passo

global h

M=810; %kg

final_post_error=[x y]-final_pos;
post_error=norm(final_post_error);

post_e=[x;y]-[next_full_state.x;next_full_state.y];
n_pe=norm(post_e);

%Orienta��o 'ideal'
beta=atan2(-post_e(2),-post_e(1));

%Erro de orienta��o
alpha=beta-(theta+phi);

if alpha > pi
        alpha = alpha - 2*pi;
elseif alpha < -pi
        alpha = alpha + 2*pi;
end

%Energia dispon�vel por passo
E_step=E_remaining/N_steps;
%Velocidade m�xima para atingirmos o fim da traject�ria
v_max_1=E_step/(Po*h);

if v_max_1>(40/3.6) || v_max_1<0
  v_max_1=(40/3.6);
end

%Velocidade angular m�xima
v_max_2=(pi/4);
%Acelara��o m�xima
a_max = 10*h;

K1=0.02*exp(-abs(90*alpha));
K2=1; K3=10;

V=v_max_1*tanh(K1*post_error);

% Restri��o da acelara��o
if (a_max < abs((V-last_V)/h))
    V = last_V + sign(V-last_V)*a_max*h;
end

% Velocidade m�xima defenida por um sinal (evento E2)
if V>v_max
    V=v_max;  
end

Ws=v_max_2*tanh(((1+(K2*(beta/alpha)))*(tanh(K1*post_error)/post_error)*sin(alpha)+K3*tanh(alpha)));

dv=(V-last_V)/h;
dE=(M*abs(dv)*abs(V)+Po)*h;

%Energia gasta at� ao momento
E_spent=E_spent+dE;
%Energia restante
E_remaining=E_bg-E_spent;


end

