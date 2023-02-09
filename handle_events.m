function [final_post,v_max,f1,f2,f3,k,sorted_flags] = handle_events(path_x,path_y,trajectory,i,f1,f2,f3,final_post,v_max,k,sorted_flags)
% HANDLE_EVENTS - Função utilizada para lidar com os eventos E1, E2 e E3
% Inputs:
% path_x - trajectória do carro em x
% path_y - trajectória do carro em y
% trajectory - matrix com trajectória do carro e flags para o eventos
% i - indice do ponto da trajectória a ser analisado
%
% Inputs/Outputs:
% f1 - vetor com os indices onde ocorre eventos do tipo E1 (sinal stop)
% f2 - vetor com os indices onde ocorre eventos do tipo E2 (sinal de velocidade máxima)
% f3 - vetor com os indices onde ocorre eventos do tipo E3 (sinal de pedestres)
% final_post - posição final a ser atingida pelo carro
% v_max - velocidade máxima que pode ser atingida pelo carro
% k - variável incrementada para definir o tempo que o carro está parado no sinal de stop
% sorted_flags - vetor com os indices dos eventos organizados pelo tempo em
% que acontecem 

final_post=final_post;
v_max=v_max;
k=k;


if ~isempty(sorted_flags)

    if ~isempty(f1) && sorted_flags(1)==f1(1)
        
        if i<f1(1) 
            final_post=[path_x(f1(1)) path_y(f1(1))];
        elseif i>f1(1)+7
            final_post=[path_x(end) path_y(end)];
            f1(1)=[]; k=0; sorted_flags(1)=[];
        end
        
    end
    
    if ~isempty(f2) && sorted_flags(1)==f2(1)
        if i>f2(1)
            v_max=(trajectory(4,f2(1)+1)/3.6)-0.5;
            f2(1)=[]; sorted_flags(1)=[];
        else
            v_max=1000;
        end
    end
    
    if ~isempty(f3) && sorted_flags(1)==f3(1)
        
        if i<f3(1)+5
            final_post=[path_x(f3(1)+5) path_y(f3(1)+5)];
        else
            final_post=[path_x(end) path_y(end)];
            f3(1)=[]; sorted_flags(1)=[];
        end
        
    end
    
elseif isempty(sorted_flags)
    final_post=[path_x(end) path_y(end)];
end


end

