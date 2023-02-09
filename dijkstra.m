%Função que implementa o algoritmo de dijkstra
%Arguments: StartNode - No inicial
%           Grafo - representacao do grafo
%Return: d - Distancias dos nos ao startNode
%        parent - no que precede esse no no caminho

function [d, parent] = dijkstra(startNode, Grafo)
    %POR NUMA FUNÇAO
    [numRows,numCols] = size(Grafo);
    d = 1:numRows;
    parent = 1:numCols;
    %if 0 não os removi
    %if 1 já removi essa posição
    queueArray = 1:numRows;

    %Inicializar vetores
    for i = 1:numRows
        d(i) = inf;
        parent(i) = -1;
        queueArray(i) = 0;
    end
    d(startNode) = 0;

    %Flag que representa se a queue ainda tem nos
    queueFlag = 0;
    currNode = startNode;
    while (queueFlag == 0)
        min = inf;
        %Escolher o valor da queue com menor distancia
        for i = 1:numRows
            %Distancia minima e ainda não foi retirado da queue
            if ((d(i) < min) & (queueArray(i) == 0))
                min = d(i);
                currNode = i;
            end
        end
        queueArray(currNode) = 1;
        queueFlag = 1;

        %Ver vizinhos do curr node e alterar distancias
        for i = 1:numRows
            %Se a distancia do vizinho a origem é maior
            if(Grafo(currNode, i) > 0 && d(i) > (d(currNode)+Grafo(currNode, i)))
               %Alterar distancia desse vizinho do currNode
               d(i) = d(currNode)+Grafo(currNode, i);
               parent(i) = currNode; 
            end

            %Verificar se a queue esta vazia
            if(queueArray(i) == 0)
                queueFlag = 0;
            end
        end   
    end
end