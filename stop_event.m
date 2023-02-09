function [final_post,V,Ws,k,p,f4,f4_act,sorted_flags] = stop_event(trajectory,i,j,k,p,path_x,path_y,f4_act,f1,f4,t_stop,final_post,V,Ws,sorted_flags)
%STOP_EVENT Função utilizada para os eventos em que é preciso parar o carro
%
% Inputs:
% trajectory - matrix com trajectória do carro e flags para o eventos
% i - indice do ponto da trajectória pré-defenida a ser analisado
% j - indice do ponto da trajectória 
% enquanto o pedestre passa
% path_x - trajectória do carro em x
% path_y - trajectória do carro em y
% f1 - vetor com os indices onde ocorre eventos do tipo E1 (sinal stop)
% t_stop - tempo para estar parado no sinal stop
% final_pos - posição final que queremos que o carro esteja
%
% Inputs/Outputs:
% f4_act - evento E4 ativo 
% V - Velocidade linear do carro
% Ws - Velocidade angular do carro
% k - variável incrementada para definir o tempo que o carro está parado no sinal de stop
% p - variável incrementada para definir o tempo que o carro está parado
% f4 - vetor com os indices onde ocorre eventos do tipo E4 (passagem de pedestre)
% sorted_flags - vetor com os indices dos eventos organizados pelo tempo em
% que acontecem 

final_post=final_post;
V=V;
Ws=Ws;
k=k; p=p;
global h

if ~isempty(f1) && i>f1(1) && k<=(t_stop/h)
    V=0;
    Ws=0;
    k=k+1;
end

if ~isempty(f4) 
    if f4_act && (j*h)>=trajectory(5,f4(1)+1) && p<=(trajectory(4,f4(1)+1)/h)
        V=0;
        Ws=0;
        p=p+1;
    elseif f4_act && i>f4(1)
        final_post=[path_x(end) path_y(end)];
        p=0; f4_act=0; 
    end
end

if ~isempty(f4) && ~isempty(sorted_flags) && i>f4(1) && sorted_flags(1)==f4(1)
    sorted_flags(1)=[];
    f4(1)=[];
end

end

