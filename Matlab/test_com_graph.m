



ws = [-1 1 -1 1];

N = 40;
rho = 0.3;

pos = rand(2,N)*2-1;

figure(1)
plot(pos(1,:),pos(2,:),'b.','MarkerSize',20)
axis equal
axis(ws)
for k = 1:1:N
    text(pos(1,k)+0.02,pos(2,k),sprintf('%d',k))
end


M = zeros(N,N);
for i = 1:1:N
    for j = i+1:1:N
        M(i,j) = sqrt( (pos(1,i)-pos(1,j))^2 + (pos(2,i)-pos(2,j))^2);
    end
end
M = M+M';
BM = M<rho;

hold on
for i = 1:1:N
    for j = i+1:1:N
        if BM(i,j) == 1
            plot(pos(1,[i j]),pos(2,[i j]),'k-')
        end
    end
end
hold off
%%

[CG] = Tarjans(BM);
hold on
cores = distinguishable_colors(length(CG));
for k = 1:1:length(CG)
    plot(pos(1,CG(k).robots),pos(2,CG(k).robots),'o','MarkerSize',20,'LineWidth',2,'Color',cores(k,:))
end
hold off








