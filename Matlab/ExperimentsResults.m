
M2p5 = [1012,	584,	475,	440;
1027,	564,	388,	407;
1062,	612,	438,	432;
1050,	622,	494,	418;
1052,	604,	441,	476;
1054,	620,	563,	470;
1039,	708,	490,	514;
1025,	557,	619,	360;
1033,	637,	491,	421;
1040,	574,	470,	439];
% figure(100)
% boxplot(M2p5)
% grid on
% ylim([0 1200])

M0 = [1012,	497,	840,	686;
1027,	547,	382,	884;
1062,	853,	584,	783;
1050,	652,	719,	698;
1052,	593,	552,	442;
1054,	841,	384,	600;
1039,	885,	632,	713;
1025,	649,	565,	623;
1033,	680,	560,	400;
1040,	672,	556,	872];
% figure(101)
% boxplot(M0)
% grid on
% ylim([0 1200])

M4p0 = [1012,	562,	430,	514;
1027,	555,	419,	409;
1062,	590,	489,	465;
1050,	587,	505,	416;
1052,	589,	443,	400;
1054,	528,	480,	433;
1039,	608,	519,	421;
1025,	572,	431,	425;
1033,	611,	502,	384;
1040,	516,	480,	411];
% figure(102)
% boxplot(M4p0)
% grid on
% ylim([0 1200])

Minf = [1012,	548,	414,	396;
1027,	502,	422,	398;
1062,	553,	411,	362;
1050,	635,	443,	388;
1052,	558,	444,	383;
1054,	531,	428,	369;
1039,	606,	473,	371;
1025,	610,	485,	381;
1033,	614,	429,	402;
1040,	542,	435,	368];
% figure(103)
% boxplot(M4p0)
% grid on
% ylim([0 1200])

M1p5 = [1012,	601,	587,	452;
1027,	553,	391,	546;
1062,	647,	577,	556;
1050,	661,	620,	448;
1052,	580,	545,	506;
1054,	853,	390,	511;
1039,	678,	496,	542;
1025,	658,	629,	519;
1033,	676,	458,	455;
1040,	576,	558,	505];
% figure(104)
% boxplot(M1p5)
% grid on
% ylim([0 1200])


M3p5 = [1012,	585,	469,	411;
1027,	563,	391,	441;
1062,	603,	466,	442;
1050,	585,	486,	474;
1052,	599,	397,	430;
1054,	539,	473,	363;
1039,	610,	449,	384;
1025,	534,	456,	425;
1033,	561,	433,	410;
1040,	549,	453,	410];
% figure(105)
% boxplot(M3p5)
% grid on
% ylim([0 1200])


figure(200)
% MM1 = [M0(:,1),M1p5(:,1),M2p5(:,1),M3p5(:,1),M4p0(:,1),Minf(:,1)];
% MM2 = [M0(:,2),M1p5(:,2),M2p5(:,2),M3p5(:,2),M4p0(:,2),Minf(:,2)];
% MM3 = [M0(:,3),M1p5(:,3),M2p5(:,3),M3p5(:,3),M4p0(:,3),Minf(:,3)];
% MM4 = [M0(:,4),M1p5(:,4),M2p5(:,4),M3p5(:,4),M4p0(:,4),Minf(:,4)];
% rho = [0 1.5 2.5 3.5 4.0 200];
% MM1 = [M0(:,1),M1p5(:,1),M3p5(:,1),Minf(:,1)];
% MM2 = [M0(:,2),M1p5(:,2),M3p5(:,2),Minf(:,2)];
% MM3 = [M0(:,3),M1p5(:,3),M3p5(:,3),Minf(:,3)];
% MM4 = [M0(:,4),M1p5(:,4),M3p5(:,4),Minf(:,4)];
% rho = [0 1.5 3.5 200];
subplot(2,2,1)
boxplot(MM1,rho)
% plot(mean(MM1),'b.-','MarkerSize',15)
% boxplot(mean(MM1),rho,'PlotStyle','compact')
title('1 robot')
xlabel('rho')
ylabel('Total task time')
% ylim([0 1200])
grid on
subplot(2,2,2)
boxplot(MM2,rho)
% plot(mean(MM2),'b.-','MarkerSize',15)
% boxplot(mean(MM2),rho,'PlotStyle','compact')
title('2 robot')
xlabel('rho')
ylabel('Total task time')
% ylim([0 1200])
grid on
subplot(2,2,3)
boxplot(MM3,rho)
% plot(mean(MM3),'b.-','MarkerSize',15)
% boxplot(mean(MM3),rho,'PlotStyle','compact')
title('3 robot')
xlabel('rho')
ylabel('Total task time')
% ylim([0 1200])
grid on
subplot(2,2,4)
boxplot(MM4,rho)
% plot(mean(MM4),'b.-','MarkerSize',15)
% boxplot(mean(MM4),rho,'PlotStyle','compact')
title('4 robot')
xlabel('rho')
ylabel('Total task time')
% ylim([0 1200])
grid on


Data = [struct('number_of_robots',1,'number_of_runs',10,'rho_vec',[0 1.5 2.5 3.5 4.0 200],'Matrix_seconds',MM1), ...
    struct('number_of_robots',2,'number_of_runs',10,'rho_vec',[0 1.5 2.5 3.5 4.0 200],'Matrix_seconds',MM2), ...
    struct('number_of_robots',3,'number_of_runs',10,'rho_vec',[0 1.5 2.5 3.5 4.0 200],'Matrix_seconds',MM3), ...
    struct('number_of_robots',4,'number_of_runs',10,'rho_vec',[0 1.5 2.5 3.5 4.0 200],'Matrix_seconds',MM4)];

% save('Data.mat','Data')


figure(300)
MM1 = [M0(:,1),M1p5(:,1),M3p5(:,1),Minf(:,1)];
MM2 = [M0(:,2),M1p5(:,2),M3p5(:,2),Minf(:,2)];
MM3 = [M0(:,3),M1p5(:,3),M3p5(:,3),Minf(:,3)];
MM4 = [M0(:,4),M1p5(:,4),M3p5(:,4),Minf(:,4)];
rho = [0 1.5 3.5 200];
subplot(1,3,1)
boxplot(MM2,rho)
title('2 robot')
xlabel('rho')
ylabel('Total task time')
ylim([0 1200])
grid on
subplot(1,3,2)
boxplot(MM3,rho)
title('3 robot')
xlabel('rho')
ylabel('Total task time')
ylim([0 1200])
grid on
subplot(1,3,3)
boxplot(MM4,rho)
% plot(mean(MM4),'b.-','MarkerSize',15)
% boxplot(mean(MM4),rho,'PlotStyle','compact')
title('4 robot')
xlabel('rho')
ylabel('Total task time')
ylim([0 1200])
grid on
