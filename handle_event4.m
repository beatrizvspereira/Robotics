function [final_post,f4_act] = handle_event4(V,i,j,f4,trajectory,curr_post,path_x,path_y,final_post,f4_act,sorted_flags)
%HANDLE_EVENT4 Função utilizada para lidar com o evento 4 

global h
final_post=final_post;
f4_act=f4_act;

if ~isempty(f4) && ~isempty(sorted_flags) && sorted_flags(1)==f4(1)
    if i<f4(1) && i>f4(1)-50
        d=norm(trajectory(1:2,f4(1)+1)-curr_post); %calcular a distância para o sitio onde o pedreste vai passar
        dt=d/V; % obter uma aproximação do intervalo de tempo tendo em conta a velocidade
        tmp=(j*h)+dt; % tempo previsto que passagem nesse ponto 
        
        if trajectory(5,f4(1)+1)-tmp<5 && trajectory(5,f4(1)+1)-tmp>0 % verificar se passa nesse sitio perto do instante dado para o inicio do evento
            final_post=[path_x(f4(1)) path_y(f4(1))];
            f4_act=1;
        end
    end
end

end

