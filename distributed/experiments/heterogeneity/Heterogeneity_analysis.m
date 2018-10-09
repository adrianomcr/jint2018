


clear all
% close all

M1 = dlmread('./exp_heterogeneity_A2_2_robs.txt');
M2 = dlmread('./exp_heterogeneity_A2_3_robs.txt');
M3 = dlmread('./exp_heterogeneity_A2_4_robs.txt');

% Each column of M1:
% loop_time, atri_time heur_time task_time_0 task_time_1 cpp_time total_time
% Each column of M2:
% loop_time, atri_time heur_time task_time_0 task_time_1 task_time_2 cpp_time total_time
% Each column of M3:
% loop_time, atri_time heur_time task_time_0 task_time_1 task_time_2 task_time_3 cpp_time total_time

% ---------- ---------- ---------- DATA ---------- ---------- ---------- %
% Vs = [1.0, 1.5, 2.5, 3.5]  # search speeds (rad/s) maximum is pi/2
% Vd = [0.5, 1.0, 1.5, 2.5]  # moving speeds (m/s)
% ---------- ---------- ---------- ---- ---------- ---------- ---------- %


M(:,:,1) = [M1(:,[4 5 1 2 3 6]), zeros(20,2)];
M(:,:,2) = [M2(:,[4 5 6 1 2 3 7]), zeros(20,1)];
M(:,:,3) = M3(:,[4 5 6 7 1 2 3 8]);


% Plot the results when the velocities are considered
for r = 2:1:4
    
    figure(r)
    
    subplot(2,1,1)
    x = M(:,1:r,r-1);
    bar(x)
    xlabel('robot id')
    ylabel('time (s)')
    title(sprintf('Task time - %d robots',r))
    xlim([0 21])
    
    subplot(2,1,2)
    bar(var(x'))
    xlabel('robot id')
    ylabel('time (s)')
    title(sprintf('Variance - %d robots',r))
%     ylim([0, 10000])
    xlim([0 21])
    
    drawnow
    
end



%Now disregarding heterogeneuty in the plan
M1x = dlmread('./exp_heterogeneity_A2_x2_robs.txt');
M2x = dlmread('./exp_heterogeneity_A2_x3_robs.txt');
M3x = dlmread('./exp_heterogeneity_A2_x4_robs.txt');

Mx(:,:,1) = [M1x(:,[4 5 1 2 3 6]), zeros(20,2)];
Mx(:,:,2) = [M2x(:,[4 5 6 1 2 3 7]), zeros(20,1)];
Mx(:,:,3) = M3x(:,[4 5 6 7 1 2 3 8]);

% Plot the results when the velocities are NOT considered
for r = 2:1:4
    
    figure(r+10)
    
    subplot(2,1,1)
    x = Mx(:,1:r,r-1);
    bar(x)
    xlabel('robot id')
    ylabel('time (s)')
    title(sprintf('Task time (no heterogeneity) - %d robots',r))
    xlim([0 21])
    
    subplot(2,1,2)
    bar(var(x'))
    xlabel('robot id')
    ylabel('time (s)')
    title(sprintf('Variance (no heterogeneity) - %d robots',r))
%     ylim([0, 10000])
    xlim([0 21])
    
    drawnow
end




