function SHOWGRAPH(QUADGRAPH,fig,GRAPH,MeetPoints,resolution)

figure,
w_s = [0 length(fig(1,:,1)) 0 length(fig(:,1,1))]*resolution;
axis (w_s)

image = (fig(:,:,1)+fig(:,:,2)+fig(:,:,3))/3;
image = flipud(image);
x = linspace(w_s(1),w_s(2),length(fig(1,:,1)));
y = linspace(w_s(3),w_s(4),length(fig(:,1,1)));
[X,Y] = meshgrid(x,y);



H = pcolor(X,Y,image);
H.LineStyle = 'none';
colormap gray
axis equal
% a*d
for k = 1:1:length(GRAPH)
    tempcxcy=QUADGRAPH.Pol_coefs(k);
    cx=tempcxcy.coef_x;
    cy=tempcxcy.coef_y;
    
    x = GRAPH(k).path;
    x = x(:,2)*resolution;
    dt = 1/length(x);
    
    %Simulate the computed polynomial
    xsim = [];
    ysim = [];
    for t = 0:dt:1
        xsim(end+1) = cx(1)*t^5+cx(2)*t^4+cx(3)*t^3+cx(4)*t^2+cx(5)*t^1+cx(6)*t^0;
        ysim(end+1) = cy(1)*t^5+cy(2)*t^4+cy(3)*t^3+cy(4)*t^2+cy(5)*t^1+cy(6)*t^0;
    end
    hold on
    plot(xsim,ysim,'b','LineWidth',2);
end

% error('Stop here to capture the image, or comment it')
warning('Stop here to capture the image, or comment it')


nodes=QUADGRAPH.node_list;

%Plotting nodes in quadratic space
hold on
for k = 1:1:length(nodes(:,1))
    plot(nodes(k,1),nodes(k,2),'*r','LineWidth',2)
    plot(nodes(k,1),nodes(k,2),'or','LineWidth',2)
    text(nodes(k,1)+0.1,nodes(k,2)+0.1,sprintf('%d',k),'FontSize',15,'color',[0 1 0])
end
title('Quadratic form Graph');
axis(w_s)

%Plotting nodes in pixel space
figure,
imshow(fig);
hold on;

handler = plot(nan, '.');

xpath=[];
ypath=[];
for k = 1:1:length(GRAPH)
    tmp=GRAPH(k).path;
    xpath=[xpath; tmp(:,1)];
    ypath=[ypath; tmp(:,2)];
end

set(handler,'XData',ypath,'YData',xpath);
drawnow;

for k = 1:1:size(MeetPoints,2)
    plot(MeetPoints(k).y,MeetPoints(k).x,'*r','LineWidth',2)
    plot(MeetPoints(k).y,MeetPoints(k).x,'or','LineWidth',2)
    text(MeetPoints(k).y+0.1,MeetPoints(k).x+0.1,sprintf('%d',k),'FontSize',15,'color',[0 1 0])
end
title('Pixel form Graph');

