clc,clear,close all
load gp_task_1_data.txt
x=gp_task_1_data';

subplot(211)
plot(x(1,:),'r','linewidth',2);grid on
title('distance')
subplot(212)
plot(x(2,:),'b','linewidth',2);grid on
title('angle')