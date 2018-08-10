function graph=QUADRATICFORM(PATH,fig,VERTEXNUM,MeetPoints,resolution)


% PATH
% VERTEXNUM
% MeetPoints
% resolution


w_s = [0 length(fig(1,:,1)) 0 length(fig(:,1,1))]*resolution;

%Adds the path to the graph library
addpath('./graphutils')

graph = struct('number_nodes',[],'node_list',[],'edge_matrix',[],'complete_edge_matrix',[],'map_edge_matrix',[],'path_matrix',[],'w_s',[],'Pol_coefs',[]);

graph.w_s = w_s;


graph.number_nodes =VERTEXNUM ;


%Get the position of the nodes (vertices)
nodes = inf*ones(VERTEXNUM,2);
for k = 1:1:length(PATH)
    node_id = PATH(k).from;
    nodes(node_id,1) = PATH(k).path(1,2)*resolution;
    nodes(node_id,2) = w_s(4)-PATH(k).path(1,1)*resolution;
    node_id = PATH(k).To;
    nodes(node_id,1) = PATH(k).path(end,2)*resolution;
    nodes(node_id,2) = w_s(4)-PATH(k).path(end,1)*resolution;
end


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
for k = 1:1:length(PATH)
    i = PATH(k).from;
    j = PATH(k).To;
    x = PATH(k).path;
    y = w_s(4)-x(:,1)*resolution;
    x = x(:,2)*resolution;
    
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
    
    %     %Create a "structure for a polynomial" and store it in the matrix
    %     pol = struct('from',PATH(k).from,'to',PATH(k).To,'coef_x',cx,'coef_y',cy,'cost',[]);
    %     Pol_coefs(k) = pol;
    
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
    E_pol(PATH(k).from,PATH(k).To) = comp;
    
    %Create a "structure for a polynomial" and store it in the matrix
    pol = struct('from',PATH(k).from,'to',PATH(k).To,'coef_x',cx,'coef_y',cy,'cost',comp);
    Pol_coefs(k) = pol;
    
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
    map_edge_matrix(Pol_coefs(k).from,Pol_coefs(k).to) = k;
    map_edge_matrix(Pol_coefs(k).to,Pol_coefs(k).from) = k;
end
graph.map_edge_matrix = map_edge_matrix;


graph.path_matrix = Paths;
graph.edge_matrix = E_pol;


% Add the polinomials to the structure
graph.Pol_coefs = Pol_coefs;




