%Depots = [41 69 6 16 50 25 8 44 34 55 51 74 43 24 57 13 18 17 31 71]

R_1 = [440.7 16.8 38.2 365.7 334.8 135.2 481.5 95.2 159.5 521.6 76.2 151.9 447.4 182.3 70.5 504.4 332.2 359.8 224.5 28.8];

R_2 = [203.4 205.5 338.1 315.8 302.6 24.3 276.6 126.2 86.5 246.8 282.0 152.2 210.3 16.8 317.1 81.9 218.1 189.1 132.7 233.7];
meet_2 = [1 1 1 1 1 0 1 1 0 1 1 0 0 0 0 0 0 1 1 1];

R_3 = [100.2 122.7 38.8 212.0 159.4 215.4 88.7 315.1 160.3 234.1 24.9 47.2 256.4 17.6 102.8 187.0 248.2 51.4 102.7 29.8];
meet_3 = [1 1 0 3 2 5 1 5 3 4 0 0 4 0 1 3 5 0 1 0];

%R_4 = [100.6 126.7 39.4 68.4 107.6 184.7 32.8 142.8 66.9 142.7 142.5 127.1 159.8 17.9 103.5 73.1 67.7 52.1 102.8 29.9];
R_4 = [100.6 126.7 39.4 68.4 107.6 24.7 32.8 142.8 66.9 142.7 142.5 127.1 159.8 17.9 103.5 73.1 67.7 52.1 102.8 29.9];
meet_4 = [3 3 0 2 3 0 0 4 1 5 5 2 5 0 2 3 2 1 2 0];







% length(R_1)
% length(R_2)
% length(R_3)
% length(R_4)


figure(1)
x = [R_1;R_2;R_3;R_4];
boxplot(x','Labels',{'M = 1','M = 2','M = 3','M = 4'})


title('Variation of object position','FontSize',15)
% xlabel('Number of robots','FontSize',15)
ylabel('Time (s)','FontSize',15)
% grid on


% mean(R_1)
% mean(R_2)
% mean(R_3)
% mean(R_4)


% Vs = [pi / 1.5, pi / 2, pi / 3, pi / 2]
% Vd = [0.4, 0.55, 0.5, 0.4]
% Vs = Vs/1.1
% Vd = Vd/1.1
% Vd = [0.36, 0.50, 0.45, 0.36]
% Vs = [1.90, 1.40, 0.95, 1.40]