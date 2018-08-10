



figure(3)

for k = 1:1:1800

    E0 = dlmread('./../distributed/text/visited_0.txt');
    E1 = dlmread('./../distributed/text/visited_1.txt');
    E2 = dlmread('./../distributed/text/visited_2.txt');
    % E3 = dlmread('./../distributed/text/visited_3.txt');

    % E = [E0;E1];
    E = [E0;E1;E2];
    % E = [E0;E1;E2;E3];
    % 

%     hist(E,39)
%     xlim([1, 39])
    hist(E,81)
    xlim([1, 81])
    ylim([0, 3])
    drawnow
    pause(1)
end
% hist(E0,39)
% xlim([1, 39])
figure(3)
plot(E)


% R = 2
% n = 36
% m = n*(n-1)
% 
% var = R*m+1
% res = R + (n-R) + R*(n-R) + R

%%
pos1 = [1,1;1,3;1,5]'; pos1 = fliplr(pos1);
cor1 = ['bo';'ro';'go'];
cor1 = ['b-';'r-';'g-'];
cor11 = [0 0 1; 1 0 0; 0 1 0];
pos2 = [3,0.5;3,1.5;3,2.5;3,3.5;3,4.5;3,5.5]'; pos2 = fliplr(pos2);

radius = 0.3;
circx = radius*cos(0:2*pi/50:2*pi);
circy = radius*sin(0:2*pi/50:2*pi);

figure(1)
w_s = [0 4 0 6];
plot(w_s([1 2 2 1 1]),w_s([3 3 4 4 3]),'k')
axis equal
axis (w_s+[-1 1 -1 1])
hold on
for r = 1:1:3
%     plot(pos1(1,r),pos1(2,r),cor1(r,:),'MarkerSize',50,'LineWidth',3)
    plot(circx+pos1(1,r),circy+pos1(2,r),cor1(r,:),'MarkerSize',50,'LineWidth',3)
    text(pos1(1,r)-0.04,pos1(2,r),sprintf('$r_%d$',r),'Color',cor11(r,:),'Interpreter','latex','FontSize',15)
end
% plot(1,1,'bo','MarkerSize',50,'LineWidth',3)
% plot(1,3,'ro','MarkerSize',50,'LineWidth',3)
% plot(1,5,'go','MarkerSize',50,'LineWidth',3)

for t = 1:1:6
%     plot(pos2(1,t),pos2(2,t),'ko','MarkerSize',50,'LineWidth',3)
    plot(circx+pos2(1,t),circy+pos2(2,t),'k-','MarkerSize',50,'LineWidth',3)
    text(pos2(1,t)-0.04,pos2(2,t),sprintf('$t_%d$',t),'Interpreter','latex','FontSize',15)
end
% plot(3,0.5,'ko','MarkerSize',50,'LineWidth',3)
% plot(3,1.5,'ko','MarkerSize',50,'LineWidth',3)
% plot(3,2.5,'ko','MarkerSize',50,'LineWidth',3)
% plot(3,3.5,'ko','MarkerSize',50,'LineWidth',3)
% plot(3,4.5,'ko','MarkerSize',50,'LineWidth',3)
% plot(3,5.5,'ko','MarkerSize',50,'LineWidth',3)


for r = 1:1:3
    for t = 1:1:6
        
        vec = pos2(:,t)-pos1(:,r);
        norm_vec = norm(vec);
        
        p1 = pos1(:,r) + radius*vec/norm_vec;
        p2 = pos2(:,t) - radius*vec/norm_vec;
        %plot([p1(1) p2(1)],[p1(2) p2(2)],'k-','LineWidth',1)
        quiver(p1(1),p1(2),p2(1)-p1(1),p2(2)-p1(2),'k','LineWidth',1,'AutoScale','off','MaxHeadSize',0.2)
    end
end


% text(2-0.04,5.4,'$X_{1,1}$','Interpreter','latex','FontSize',15)
% text(2-0.1,4.85,'$X_{1,2}$','Interpreter','latex','FontSize',15)
% text(2-0.04,0.85,'$X_{3,6}$','Interpreter','latex','FontSize',15)
text(2-0.04,5.4,'$c_{1,1}$','Interpreter','latex','FontSize',15)
text(2-0.1,4.85,'$c_{1,2}$','Interpreter','latex','FontSize',15)
text(2-0.04,0.85,'$c_{3,6}$','Interpreter','latex','FontSize',15)

hold off



