


close all
clear all


load('./Data.mat')



figure(1)
subplot(2,2,1)
boxplot(Data(1).Matrix_seconds,Data(1).rho_vec)
title('1 robot')
xlabel('rho')
ylabel('Total task time')
% ylim([0 1200])
grid on
subplot(2,2,2)
boxplot(Data(2).Matrix_seconds,Data(2).rho_vec)
title('2 robot')
xlabel('rho')
ylabel('Total task time')
% ylim([0 1200])
grid on
subplot(2,2,3)
boxplot(Data(3).Matrix_seconds,Data(3).rho_vec)
title('3 robot')
xlabel('rho')
ylabel('Total task time')
% ylim([0 1200])
grid on
subplot(2,2,4)
boxplot(Data(4).Matrix_seconds,Data(4).rho_vec)
title('4 robot')
xlabel('rho')
ylabel('Total task time')
% ylim([0 1200])
grid on

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.2, 0.2, 0.6, 0.8]);




figure(2)
subplot(1,3,1)
boxplot(Data(2).Matrix_seconds(:,[1 2 3 5]),Data(2).rho_vec([1 2 3 5]))
title('2 robot')
xlabel('rho')
ylabel('Total task time')
ylim([0 1200])
grid on
subplot(1,3,2)
boxplot(Data(3).Matrix_seconds(:,[1 2 3 5]),Data(3).rho_vec([1 2 3 5]))
title('3 robot')
xlabel('rho')
ylabel('Total task time')
ylim([0 1200])
grid on
subplot(1,3,3)
boxplot(Data(4).Matrix_seconds(:,[1 2 3 5]),Data(4).rho_vec([1 2 3 5]))
title('4 robot')
xlabel('rho')
ylabel('Total task time')
ylim([0 1200])
grid on

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.1, 0.1, 0.8, 0.5]);


%%
i = 2000;
j = 1.06^(1/12);
I = 0;
for k = 1:1:12*20
    I = I*j + i;
end

I/1000;



%%
