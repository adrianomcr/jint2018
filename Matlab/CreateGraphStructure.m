
close all;
clear all; clc;

% Choose which graph to use
% NAME = '35';
NAME = 'A1';


fig = imread(sprintf('./pixel_files/Map%s.jpg',NAME));
load(sprintf('./pixel_files/GraphMap%s.mat',NAME));
%Obs: This structure must contain:
%Path_l = 
%1x83 struct array with fields:
%    path
%    from
%    To
%MeetPoints = 
%1x74 struct array with fields:
%    x
%    y
%    indexinPath

if 1
    flag = 1;
    while flag == 1
        flag = 0;

        %Loop to detect a short edge
        for e = 1:1:length(Path_l)
            if length(Path_l(e).path(:,1)) < 5
                flag = 1;
                break
            end
        end

        %Remove the short edge
        if (flag == 1)
            id1 = Path_l(e).from;
            id2 = Path_l(e).To;
            newid = min([id1,id2]); % id to be kept
            delid = max([id1,id2]); % id to be removed
            Path_l(e) = [];
            %Replace the ids of the other edges
            for e = 1:1:length(Path_l)
                if Path_l(e).from == delid
                    Path_l(e).from = newid;
                end
                if Path_l(e).To == delid
                    Path_l(e).To = newid;
                end
            end
            %Shift the ids bigger than newid
            for e = 1:1:length(Path_l)
                if Path_l(e).from > delid
                    Path_l(e).from = Path_l(e).from-1;
                end
                if Path_l(e).To > delid
                    Path_l(e).To = Path_l(e).To-1;
                end
            end

        end

    end

end % if 0/1






%Transform pixels to meters
resolution = 0.02;

w_s = [0 length(fig(1,:,1)) 0 length(fig(:,1,1))]*resolution;
axis (w_s)



figure(1)
image = (fig(:,:,1)+fig(:,:,2)+fig(:,:,3))/3;
image = flipud(image);
x = linspace(w_s(1),w_s(2),length(fig(1,:,1)));
y = linspace(w_s(3),w_s(4),length(fig(:,1,1)));
[X,Y] = meshgrid(x,y);
H = pcolor(X,Y,image);
H.LineStyle = 'none';
colormap gray
axis equal



%Adds the path to the graph library
addpath('./graphutils')

graph = struct('number_nodes',[],'node_list',[],'edge_matrix',[],'complete_edge_matrix',[],'complete_SP_matrix',[],'map_edge_matrix',[],'path_matrix',[],'w_s',[],'Pol_coefs',[]);

graph.w_s = w_s;


%Get the position of the nodes (vertices)
for k = 1:1:length(Path_l)
    node_id = Path_l(k).from;
    nodes(node_id,1) = Path_l(k).path(1,2)*resolution;
    nodes(node_id,2) = w_s(4)-Path_l(k).path(1,1)*resolution;
    node_id = Path_l(k).To;
    nodes(node_id,1) = Path_l(k).path(end,2)*resolution;
    nodes(node_id,2) = w_s(4)-Path_l(k).path(end,1)*resolution;
end
graph.number_nodes = length(nodes(:,1));


graph.node_list = nodes;


path = struct('path',[]);
E = [];
paths = path;
k = 0;
for i = 1:1:graph.number_nodes
    for j = 1:1:graph.number_nodes
        if (i ~= j)
            k = k+1;% for k = 1:1:length(E(:,1))
            E(k,:) = [i, j];
            paths(k,1) = path;
        end
    end
end



%Matrix of lengths of the edges
E_pol = zeros(length(MeetPoints));


%Loop to generate a set of polynomials that represents each edge
for k = 1:1:length(Path_l)
    i = Path_l(k).from;
    j = Path_l(k).To;
    x = Path_l(k).path;
    y = w_s(4)-x(:,1)*resolution;
    x = x(:,2)*resolution;

%     figure(2)
%     hold on
%     plot(x,y,'y.','MarkerSize',9)
%     axis equal

    %Create matrix of regressors
    A = [];
    t = 0;
    dt = 1/length(x);
    for p = 1:1:length(x)
        A = [A; t^5 t^4 t^3 t^2 t^1 t^0];
        t = t+dt;
    end

    %Apply the minimum squares method
    cx = A\x;
    cy = A\y;
    
    %Simulate the computed polynomial
    xsim = [];
    ysim = [];
    for t = 0:dt:1
        xsim(end+1) = cx(1)*t^5+cx(2)*t^4+cx(3)*t^3+cx(4)*t^2+cx(5)*t^1+cx(6)*t^0;
        ysim(end+1) = cy(1)*t^5+cy(2)*t^4+cy(3)*t^3+cy(4)*t^2+cy(5)*t^1+cy(6)*t^0;
    end
    %Compute the length of a edge based on the computed polinomial
    comp = 0;
    for p = 1:1:(length(xsim)-1)
        d_comp = sqrt((xsim(p)-xsim(p+1))^2+(ysim(p)-ysim(p+1))^2);
        comp = comp + d_comp;
    end
    E_pol(Path_l(k).from,Path_l(k).To) = comp;
    
    %Create a "structure for a polynomial" and store it in the matrix
    pol = struct('from',Path_l(k).from,'to',Path_l(k).To,'coef_x',cx,'coef_y',cy,'cost',comp);
    Pol_coefs(k) = pol;
    
    %Plot the polynomial
%     figure(1)
    hold on
    plot(xsim,ysim,'b','LineWidth',2)
    hold off
%     figure(2)
%     hold on
%     plot(xsim,ysim,'b','LineWidth',2)
%     hold off
end
E_pol = E_pol + E_pol';


% Create container
G = container_set(vertex.empty());


for i = 1:1:graph.number_nodes
    x = graph.node_list(i,:);
    neig = [];
    cost = [];
    for j = 1:1:graph.number_nodes
        if(j ~= i)
            if(E_pol(i,j) ~= 0)
                neig = [neig j];
                cost = [cost E_pol(i,j)];
            end
        end
    end
    v = vertex(G.get_next_idx(), x, 0, cost, 0, neig, [], 0);
    G.add_element(v); 
end



% Complete the cost matrix (nodes not directly connected) with Dijkstra
complete_edge_matrix = zeros(graph.number_nodes,graph.number_nodes);
for i = 1:1:graph.number_nodes
    [success, CC, EE] = search_Dijkstra(i, 0, G);
    for j = 1:1:graph.number_nodes
        if (i ~= j)
            p = path;
            p.path = CC.container(j).traj_from_start;
            Paths(i,j) = p;
            complete_edge_matrix(i,j) = CC.container(j).cost_from_start;
        end
    end
end
graph.complete_edge_matrix = complete_edge_matrix;



map_edge_matrix = -1*ones(graph.number_nodes,graph.number_nodes);
for k = 1:1:length(Pol_coefs)
    map_edge_matrix(Pol_coefs(k).from, Pol_coefs(k).to) = k;
    map_edge_matrix(Pol_coefs(k).to, Pol_coefs(k).from) = k;
end
graph.map_edge_matrix = map_edge_matrix;


graph.path_matrix = Paths;
graph.edge_matrix = E_pol;


%Plotting nodes
hold on
plot(nodes(:,1),nodes(:,2),'r.','MarkerSize',25)
for k = 1:1:length(nodes(:,1))
    text(nodes(k,1)+0.2,nodes(k,2)+0.05,sprintf('%d',k),'FontSize',10,'Color',[1 0 0])
end


% Add the polinomials to the structure
graph.Pol_coefs = Pol_coefs;

axis(w_s)

% Save structure
save(sprintf('./output_structure/Original_graph_%s.mat',NAME),'graph')


hold off


CreateSearchPoints


%Load the search points files
M = dlmread(sprintf('./../distributed/maps/Map_%s_SP.txt',NAME));
figure(1)
hold on
plot(M(:,1),M(:,2),'go','MarkerSize',12-4,'LineWidth',1.5)
hold off


%In he original graph
C_sp = zeros(graph.number_nodes,graph.number_nodes);
for i = 1:1:graph.number_nodes
    for j = 1:1:graph.number_nodes
        if (i ~= j)
            p = graph.path_matrix(i,j);
            e = graph.map_edge_matrix(p.path(end-1),p.path(end));
            C_sp(i,j) = search_point_list(e).SP_number; % number of search points between the last edge of a path
        end
    end
end
graph.complete_SP_matrix = C_sp;


% Save structure (with complete_SP_matrix)
save(sprintf('./output_structure/Original_graph_%s.mat',NAME),'graph')



CreateVirtualNodes

figure(1)
axis(w_s+[-1 1 -1 1]*0.5)
figure(2)
axis(w_s+[-1 1 -1 1]*0.5)


figure(1)
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.4, 0.4, 0.5, 0.5]);
figure(2)
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.4, 0.4, 0.5, 0.5]);


