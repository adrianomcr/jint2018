

% close all;
% clear all; clc;


% NAME = '35';
% NAME = 'A1';

%Adds the path to the graph library
addpath('./graphutils');


fig = imread(sprintf('./pixel_files/Map%s.jpg',NAME));
load(sprintf('./output_structure/Original_graph_%s.mat',NAME),'graph');


number_nodes = graph.number_nodes;
node_list = graph.node_list;
edge_matrix = graph.edge_matrix;
complete_edge_matrix = graph.complete_edge_matrix;
map_edge_matrix = graph.map_edge_matrix;
path_matrix = graph.path_matrix;
w_s = graph.w_s;
Pol_coefs = graph.Pol_coefs;





% ----------  ----------  ----------  ----------  ----------  ----------
% ----------  ----------  ----------  ----------  ----------  ----------
MAX_DIST =1.5;
SP = struct('SP_number',[],'parameter_values',[]);
search_point_list = [SP];
for k = 1:1:length(Pol_coefs)
    n_SP = ceil(Pol_coefs(k).cost/MAX_DIST)-1; % Number of searching points in the edge
    search_point_list(k) = SP;
    search_point_list(k).SP_number = n_SP;
    if (n_SP ~= 0)
        d_p = 1/(n_SP+1);
        for l = 1:1:n_SP
            search_point_list(k).parameter_values(end+1) = (l-1*0)*d_p;
        end
    end
end
% Add the oriiginal nodes
flag_nodes = zeros(1,number_nodes);
for k = 1:1:length(Pol_coefs)
    if (flag_nodes(Pol_coefs(k).from) == 0)
        flag_nodes(Pol_coefs(k).from) = 1;
        search_point_list(k).SP_number = search_point_list(k).SP_number + 1;
        search_point_list(k).parameter_values = [0 search_point_list(k).parameter_values];
    end
    if (flag_nodes(Pol_coefs(k).to) == 0)
        flag_nodes(Pol_coefs(k).to) = 1;
        search_point_list(k).SP_number = search_point_list(k).SP_number + 1;
        search_point_list(k).parameter_values = [search_point_list(k).parameter_values 1];
    end
end




FILE=fopen(sprintf('Map_%s_SP.txt',NAME),'w');
FILE=fopen(sprintf('./../distributed/maps/Map_%s_SP.txt',NAME),'w');
for k = 1:1:length(search_point_list)
    for k2 = 1:1:search_point_list(k).SP_number
        t = search_point_list(k).parameter_values(k2);
        T = [t^5; t^4; t^3; t^2; t; 1];
        xsp = Pol_coefs(k).coef_x'*T;
        ysp = Pol_coefs(k).coef_y'*T;
        fprintf(FILE,'%.4f\t%.4f\t%d\n',xsp,ysp,k);
    end
end
fclose(FILE);






