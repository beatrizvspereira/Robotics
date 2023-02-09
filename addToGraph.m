%Função que adiciona um ponto válido ao grafo e calcula os vizinhos
%Arguments: Grafo - representacao do grafo
%           grafoCoord - coordenadas dos pontos do grafo
%           x1 - coordenada x do ponto
%           y1 - coordenada y do ponto
%           map - representação do mapa
%
%
%Return: Grafo - matriz do grafo atualizado
%        grafoCoord - coordenadas dos pontos do grafo atualizadas
%        flagEstrada - 1 se o ponto está no passeio e é invalido


function [Grafo, grafoCoord, flagEstrada] = addToGraph (Grafo, grafoCoord, x1, y1, map)
flagEstrada = 0;
if (map(y1, x1) == 0)
    flagEstrada = 1;
%Se não é estrada
else
    %Ver a quais pontos do grafo consegue chegar
        %Ver todos onde se consegue ligar?
    grafoCoord(size(grafoCoord,1)+1,1) = x1;
    grafoCoord(size(grafoCoord,1),2) = y1;
    Grafo(size(Grafo,1)+1, :) = -1;
    Grafo(:, size(Grafo,2)+1) = -1;
    for i = 1:57
        %Escolher os pontos num raio de 100 pixeis
        if (sqrt((x1-grafoCoord(i, 1))^2+(y1-grafoCoord(i, 2))^2) < 60)
            x = [x1 grafoCoord(i, 1)];
            y = [y1 grafoCoord(i, 2)];
            
            %Interpolar a reta entre os dois pontos
            h = 0.01;
            npt = length(x);       
            nvia = [0:1:npt-1];
            xq = pchip(nvia,x);
            yq = pchip(nvia,y);
            time = [0:h:npt-1];
            xq = fnval(xq, time);
            yq = fnval(yq, time);
            yq = round(yq);
            xq = round(xq);
            Inv = 0;

            %Verificar validade das ligacoes
            for j = 1:size(xq, 2)
                if (map(yq(1,j), xq(1,j)) == 0)
                    Inv = 1;     
                end
            end

            %Se a ligação e valida
            if (Inv == 0)
                %Adicionar ao grafo 
                Grafo(size(Grafo,1), size(Grafo,2)) = 0;
                Grafo(size(Grafo,1), i) = sqrt((x1-grafoCoord(i, 1))^2+(y1-grafoCoord(i, 2))^2);
                Grafo(i, size(Grafo,1)) = sqrt((x1-grafoCoord(i, 1))^2+(y1-grafoCoord(i, 2))^2);
            end
        end
    end
end