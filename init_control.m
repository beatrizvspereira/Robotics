function [path_x,path_y,f1,f2,f3,f4,t_stop,E_spent,E_bg,j,k,p,E_remaining,Po,N_steps,v_max,f4_act,final_post,car_control,sorted_flags] = init_control(trajectory)
%INIT_CONTROL - Inicialização das variáveis a ser utilizadas no controlo do
%carro e para lidar com eventos

path_x=trajectory(1,2:end);
path_y=trajectory(2,2:end);
flags=trajectory(3,2:end);

f1=find(flags==1);
t_stop=trajectory(4,f1+1);

f2=find(flags==2);
f3=find(flags==3);
f4=find(flags==4);

unsorted_flags=[f1 f2 f3 f4];
sorted_flags=sort(unsorted_flags);

E_spent(1)=0;
E_bg=800000;
j=2;
E_remaining=E_bg;
Po=10;
N_steps= length(trajectory(1,:))*2;
k=0; p=0;
v_max=1000;
f4_act=0;
final_post=[path_x(end) path_y(end)];
car_control(1).V=0;
car_control(1).Ws=0;

end

