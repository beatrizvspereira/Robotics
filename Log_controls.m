function [car_control] = Log_controls(V,Ws,car_control)
%LOG_CONTROLS Summary of this function goes here
%   Detailed explanation goes here

car_control(end+1).V=V;
car_control(end).Ws=Ws;

end

