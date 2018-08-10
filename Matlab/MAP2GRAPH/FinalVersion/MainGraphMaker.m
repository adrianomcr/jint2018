% THIS CODE IS DEVELOPED BY REZA JAVANMARD ALITAPPEH BY THE HELP OF KOSSAR
% JEDDISARAVI AND ADRIANO, WE ALSO USED A GRAPH LIBRARY FROM Carnegie Mellon University
%
% E-MAIL: RezaJavanmard64@gmail.com
% 31/08/2018
%
% THE CODE GETS AN INPUT IMAGE AND CONVERTS INTO A GRAPH; THE NODES IN THIS
% GRAPH ARE MEETPOINTS AND ENDPOINT IN THE GVD
% AND THE OUTPUT IS TWO GRAPHS IN FORM OF: PIXEL PATH AND QUADRATIC CURVES
%
%------------------------------------------------------------------------------------
%
% PARAMETERS:
%
%  INPUTS:---------------------------------------
%  ** INPUT IMAGE:  i.e. InputImage='Map8_2.jpg';
%     THE INPUT IMAGE CAN BE .JPG OR .PNG OR .BMP ETC
%
%  ** RADIUS: IT IS RADIUS OF THE ROBOT, THIS PARAMETER IS USED TO COMPUTE CONFIGURATION
%     FREE SPACE, IN CASE IF THE GRAPH IS DISCONNECTED IT MEANS ROBOT'S RADIUS
%     IS BIG AND IT CAN NOT PASS THROUGH A NARROW CORRIDOR OR DOOR IN THE
%     MAP
%
%  ** resolution: THIS PARAMETER IS USED TO CONVERT IMAGE SPACE INTO REAL
%     SPAEC I.E. METER
%
%  OUTPUTS:--------------------------------------
%  ** GRAPH: CONTAINS NODES AND PATH BETWEEN NODES(IN PIXEL)
%     path : n*2 matrix for x and y 
%     from : the corresponding start node
%     to :   the corresponding end node
%     
%  ** QUADGRAPH: CONTAINS THE GRAPH IN THE FORM OF QUADRATIC FUNCTION FOR
%     EACH EDGE
%     number_nodes: number of nodes in the constructed graph
%     node_list: location of the nodes
%     edge_matrix: Adjacency matrix of the graph
%     path_matrix: The Path between all the nodes
%     w_s :  size of the work space
%     Pol_coefs:  coefficent of the quadratic form of the edges
%     --------------------------------------------  
% NOTES:
%
%   * for the sake of clarity the function SHOWGRAPH shows how you can
%   access tot the structures (GRAPH, QUADGRAPH)
%   * If you work in other languages i.e Python, you can easily save the
%   output in .Mat file and load it in Python.
%------------------------------------------------------------------------------------


%%
close all;
clear all;
clc;
%

% ---------<input parameter>--------

% InputFileImage='Map8_2.jpg'; % input image, .jpg or .png
InputFileImage='MapA1.jpg'; % input image, .jpg or .png
ROBOTRADIUS=12; % Rotot Radius

%Transform pixels to meters for the quadratic form
resolution = 0.02;

global SHOWFIGFLAG;
global SHOWGVDFLAG;

SHOWGVDFLAG=1*0; % to show GVD construction, it takes time
SHOWFIGFLAG=1; % to show output figure
% ----------------------------------


%%

% reading the image
INPUTIMAGE=imread(InputFileImage);

if SHOWFIGFLAG
    figure, imshow(INPUTIMAGE); title('Input Map');
end

% Computing GVD of the input image
GVDMAP=CREATEGVD(INPUTIMAGE,ROBOTRADIUS);

if SHOWFIGFLAG
    figure, imshow(GVDMAP); title('GVD');
end

% Convert GVD structure to a graph representation
[EdgeLabel,vertex]=CONVERT2GRAPH(GVDMAP);

% Create GVD based pixel form Graph
[GRAPH MeetPoints]=CREATEGRAPH(EdgeLabel,vertex);

% Create Quadtaric form of the graph
VERTEXNUM=size(vertex,2);
QUADGRAPH=QUADRATICFORM(GRAPH,INPUTIMAGE,VERTEXNUM,MeetPoints,resolution);


% SHOWGRAPH show you how to access to the structures: QUADGRAPH and GRAPH
    SHOWGRAPH(QUADGRAPH,INPUTIMAGE,GRAPH,MeetPoints,resolution)


    
save('saveAll.mat')
%%

if (strcmp(InputFileImage,'MapA1.jpg'))
    Path_l = GRAPH;
    save('GraphMapA1.mat', 'MeetPoints', 'Path_l')
end

%%

MIGUE = 1;
if MIGUE

    load('saveAll.mat')

    GRAPH(24);
    GRAPH(49);

    node = GRAPH(81).path(end,:);
    minDist = norm(GRAPH(24).path(1,:)-node);
    id = 1;
    for k = 2:1:length(GRAPH(24).path(:,1))
        if norm(GRAPH(24).path(k,:)-node) < minDist
            minDist = norm(GRAPH(24).path(k,:)-node);
            id = k;
        end
    end

    ggg = struct('path',[],'from',0,'To',0);
    GRAPH(end+1) = ggg;
    GRAPH(end).path = GRAPH(24).path(1:id,:);
    GRAPH(end).from = GRAPH(24).from;
    GRAPH(end).To = 74;
    GRAPH(end+1) = ggg;
    GRAPH(end).path = GRAPH(24).path((id+1):end,:);
    GRAPH(end).from = 74;
    GRAPH(end).To = GRAPH(24).To;


    node = GRAPH(68).path(end,:);
    minDist = norm(GRAPH(49).path(1,:)-node);
    id = 1;
    for k = 2:1:length(GRAPH(49).path(:,1))
        if norm(GRAPH(49).path(k,:)-node) < minDist
            minDist = norm(GRAPH(49).path(k,:)-node);
            id = k;
        end
    end

    ggg = struct('path',[],'from',0,'To',0);
    GRAPH(end+1) = ggg;
    GRAPH(end).path = GRAPH(49).path(1:id,:);
    GRAPH(end).from = GRAPH(49).from;
    GRAPH(end).To = 63;
    GRAPH(end+1) = ggg;
    GRAPH(end).path = GRAPH(49).path((id+1):end,:);
    GRAPH(end).from = 63;
    GRAPH(end).To = GRAPH(49).To;


    GRAPH([24, 49]) = [];

%     find_this_id = 74;
%     for k = 1:1:length(GRAPH)
%         if GRAPH(k).from == find_this_id || GRAPH(k).To == find_this_id
%             k
%         end
%     end



    Path_l = GRAPH;
    save('GraphMapA1.mat', 'MeetPoints', 'Path_l')
end %MIGUE


% close all
% QUADGRAPH=QUADRATICFORM(GRAPH,INPUTIMAGE,VERTEXNUM,MeetPoints,resolution);
% SHOWGRAPH(QUADGRAPH,INPUTIMAGE,GRAPH,MeetPoints,resolution)