
clear all
% close all

M1 = dlmread('./exp_A2_2_robs.txt');
M2 = dlmread('./exp_A2_3_robs.txt');
M3 = dlmread('./exp_A2_4_robs.txt');


% Each column of M1:
% loop_time, atri_time heur_time task_time_0 task_time_1 cpp_time total_time
% Each column of M2:
% loop_time, atri_time heur_time task_time_0 task_time_1 task_time_2 cpp_time total_time
% Each column of M3:
% loop_time, atri_time heur_time task_time_0 task_time_1 task_time_2 task_time_3 cpp_time total_time

% ---------- ---------- ---------- DATA ---------- ---------- ---------- %
% Vs = [1.5, 1.3, 1.0, 1.5]  # search speeds (rad/s) maximum is pi/2
% Vd = [0.4, 0.55, 0.5, 0.4]  # moving speeds (m/s)
% ---------- ---------- ---------- ---- ---------- ---------- ---------- %


% Reorder columns
M(:,:,1) = [M1(:,[4 5 1 2 3 6 7]), zeros(20,2)];
M(:,:,2) = [M2(:,[4 5 6 1 2 3 7 8]), zeros(20,1)];
M(:,:,3) = M3(:,[4 5 6 7 1 2 3 8 9]);

% Plot 
for r = 2:1:4
    
    figure(r)
    
    subplot(2,1,1)
    x = M(:,1:r,r-1);
    bar(x)
    xlabel('robot id')
    ylabel('time (s)')
    title(sprintf('Task time - %d robots',r))
    xlim([0, 21])
    
    subplot(2,1,2)
    bar(var(x'))
    xlabel('robot id')
    ylabel('time (s)')
    title(sprintf('Variance - %d robots',r))
    xlim([0, 21])
    
    drawnow
end


% Capture computation times
loop_time = zeros(20,3);
atri_time = zeros(20,3);
heu_time = zeros(20,3);
cpp_time = zeros(20,3);
tot_tot_time = zeros(20,3);
for r = 2:1:4    
    loop_time(:,r-1) = M(:,r+1,r-1);
    atri_time(:,r-1) = M(:,r+2,r-1);
    heu_time(:,r-1) = M(:,r+3,r-1);
    cpp_time(:,r-1) =  M(:,r+4,r-1);
    tot_tot_time(:,r-1) = M(:,r+5,r-1);
end
exec_time(:,:,1) = loop_time;
exec_time(:,:,2) = atri_time;
exec_time(:,:,3) = heu_time;
exec_time(:,:,4) = cpp_time;
exec_time(:,:,5) = tot_tot_time;

% Plot computation times
for f = 1:1:5
    figure(f+200)
    boxplot(exec_time(:,:,f), [2, 3, 4])
    grid on
    
    xlabel('Number of robots')
    ylabel('Computation time (s)')
    
    switch(f)
        case 1
            title('Loop time (loop whth LP)')
        case 2
            title('Attribution time (MILP)')
        case 3
            title('Total heuristic time')
        case 4
            title('All CPPs time')
        case 5
            title('Total time')
    end
    
    drawnow
    
end

