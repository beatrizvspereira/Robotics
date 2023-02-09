function trajectory = trajectoryGen(app)
    %Matriz de adjacencias
    load 'grafo';
    %Coordenadas dos pontos do grafo
    load 'grafoCoord';

    %Upload Imagem Mapa
    map = imread('MapBinFinal.bmp');
    %Converter para binário
    map = imcomplement(map);
    
    imshow(map)
    hold on;
    
    %Escolher ponto inicial
    [x1,y1] = ginput(1);
    x1 = round(x1);
    y1 = round(y1);

    %Escolher ponto final
    [x2,y2] = ginput(1);
    x2 = round(x2);
    y2 = round(y2);

    flagEstradaEnd = 0;
    flagEstrada = 0;
    flagGrafoS = 0;
    flagGrafoE = 0;

    %Verificar se os pontos escolhidos ja fazem parte do grafo
    for i = 1:57
        if(grafoCoord(i,1) == x1 & grafoCoord(i,2) == y1)
            startNode = i;
            flagGrafoS = 1;
        end
        if (grafoCoord(i,1) == x2 & grafoCoord(i,2) == y2)
            endNode = i;
            flagGrafoE = 1;
        end
    end

    %Verificar se é estrada 
    %adicionar novos pontos ao grafo
    if (flagGrafoS == 0)
        [Grafo, grafoCoord, flagEstrada] = addToGraph (Grafo, grafoCoord, x1, y1, map);
        startNode = 58;
        if(flagGrafoE == 0)
           [Grafo, grafoCoord, flagEstradaEnd] = addToGraph (Grafo, grafoCoord, x2, y2, map);
           endNode = 59;
        end
    elseif(flagGrafoE == 0)
        [Grafo, grafoCoord, flagEstradaEnd] = addToGraph (Grafo, grafoCoord, x2, y2, map);
        endNode = 58;
    end
    
    %Se não é estrada
    if (flagEstrada == 1 || flagEstradaEnd == 1)
        %Informar e imprimir o ponto escolhido 
        disp('Ponto Inválido -> Passeio');
        imshow(map)
        hold on
        plot(x1, y1, 'ro');
        plot(x2, y2, 'ro');
        %Retornar de acordo
        trajectory = -1;
        return;

    %Se é estrada
    else

        %Correr o dijkstra sobre o grafo
        [d, parent] = dijkstra(startNode, Grafo);

        %Todos os pontos do caminho 
        caminho = zeros(1, 57, 'uint16');
        currNode = endNode;
        i = 2;
        caminho(1) = endNode;

        %Se o caminho tem um custo superior a 10000 é invalido
        if(d(endNode) > 10000)
            disp('Caminho inválido');
        end

        %Guardar caminho
        while (currNode ~= startNode)

            caminho(i) = parent(currNode); 

            currNode = parent(currNode); 
            i = i+1;
        end


        %Coordenadas dos pontos do caminho
        x = zeros(1);
        y = zeros(1);
        pontoNum = zeros(1);
        counter = 0;

        %Criar o caminho e as suas coordenadas
        for i = 30:-1:1
            if (caminho(1,i) ~= 0)
                counter = counter + 1;
                x(counter) = grafoCoord(caminho(1,i), 1);
                y(counter) = grafoCoord(caminho(1,i), 2);
                pontoNum(counter) = caminho(1,i);
            end
        end

        imshow(map)
        hold on
        %Calculo da trajectoria com os pontos do caminho 
        h = 0.01;
        npt = length(x);    % number of via points, including initial and final
        nvia = [0:1:npt-1];
        csinterp_x = pchip(nvia,x);
        csinterp_y = pchip(nvia,y);
        time = [0:h:npt-1];
        xx = fnval(csinterp_x, time);
        yy = fnval(csinterp_y, time);
        
        %Desenhar trajectória sobre o mapa
        plot(xx,yy, 'red', x, y, 'ro');
        app.EnergyGauge.Value = 30;
        app.VelocityGauge.Value = 60;
        
        %Create the return data package
        trajectory = zeros(4, size(xx,2));
        trajectory(1,:) = xx;
        trajectory(2,:) = yy;
        trajectory(3, :) = 0;
        trajectory(4, :) = 0;
        trajectory(5, :) = 0;

        %Add the events and their handler to the trajectory point
        aux = 1;
        %Percorrer caminho
        for i = 1:(size(x,2))
            if (grafoCoord(pontoNum(i), 3) ~= 0)

                %Go through the trajectoy
                for j = aux:(size(trajectory, 2))
                    if(x(i) == trajectory(1,j) & y(i) == trajectory(2,j))

                        %Add the event and handler to the trajectory
                        trajectory(3, j) = grafoCoord(pontoNum(i), 3);
                        trajectory(4, j) = grafoCoord(pontoNum(i), 4);
                        trajectory(5, j) = grafoCoord(pontoNum(i), 5);
                        aux = j;
                    end 
                end
            end
        end
    end
end