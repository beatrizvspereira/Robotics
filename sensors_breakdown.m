function sensors_breakdown(enable)
    global sensors_working sensors_state_change
    
        %avaria IMU 
        odd = 99.999;
        if (rand()*100>odd || enable(1))
            if sensors_working(1)==1
                sensors_state_change(1) = 1;
            end
            sensors_working(1)=0;
        else
            if sensors_working(1)==0
                sensors_state_change(1) = 1;
            end
            sensors_working(1)=1;
        end
        
        %GPS sinal intermitente
        odd = 30;
        if (rand()*100>odd || enable(2))
            if sensors_working(2)==1
                sensors_state_change(2) = 1;
            end
            sensors_working(2)=0;
        else
            if sensors_working(2)==0
                sensors_state_change(2) = 1;
            end
            sensors_working(2)=1;
        end
        
        %avaria do velocimetro
        odd = 99.999;
        if (rand()*100>odd || enable(3))
            if sensors_working(3)==1
                sensors_state_change(3) = 1;
            end
            sensors_working(3)=0;
        else
            if sensors_working(3)==0
                sensors_state_change(3) = 1;
            end
            sensors_working(3)=1;
        end

        

        %encoder
        odd = 99.999;
        if (rand*100>odd || enable(4))
            if sensors_working(4)==1
                sensors_state_change(4) = 1;
            end
            sensors_working(4)=0;
        else
            if sensors_working(4)==0
                sensors_state_change(4) = 1;
            end
            sensors_working(4)=1;
        end

        %potenciometer
        odd = 99.999;
        if (rand*100>odd || enable(5))
            if sensors_working(5)==1
                sensors_state_change(5) = 1;
            end
            sensors_working(5)=0;
        else
            if sensors_working(5)==0
                sensors_state_change(5) = 1;
            end
            sensors_working(5)=1;
        end

        %techometer
        odd = 99.999;
        if (rand*100>odd || enable(6))
            if sensors_working(6)==1
                sensors_state_change(6) = 1;
            end
            sensors_working(6)=0;
        else
            if sensors_working(6)==0
                sensors_state_change(6) = 1;
            end
            sensors_working(6)=1;
        end
end

