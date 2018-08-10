function [EdgeLabel,vertx]=CONVERT2GRAPH(map)

% read input patterns to detect cross points
PATTERN=readstructure();

global SHOWGVDFLAG;

[x,y]=size(map);

%------------------------------------------
% for search to find first point of path, if the first white point in
% binary image not found, return to main

map(1,1)=0;
map(x,1)=0;
map(1,y)=0;
map(x,y)=0;

[i j]=findfirstppath(map);
if (i==-1)
    disp('first point not found');
    return;
end

%----------------------------------------
k(1)=1;
l(1)=1;

showimg=uint8(map);
showimg2=showimg;

if SHOWGVDFLAG
    figure,imshow(map);
end

%% variables 
top=0;  % top of the stack
MatGraph(1,1)=0;
global vertx;  % nodes of graph
global mpstack; % stack

% first node
vertx(1,1).node(1).x=1;
vertx(1,1).node(1).y=1;
vertx(1,1).node(1).weight=0;
vertx(1,1).x=i;
vertx(1,1).y=j;
vertx(1,1).nodnum=0;
vertx(1,1).vertexnum=1;
vertx(1,1).IndexInPath=1;

pathcounter=1;
TempPath(pathcounter,1:2)=[i j];
pathcounter=pathcounter+1;

% fill the first cell of the stack
top=top+1;
mpstack(top).x=i;
mpstack(top).y=j;
mpstack(top).vertexnum=1;
nodenum(1:20)=1;
vertexnum=2;
lastflag=0;
iter=0;

global EdgeLabel;
EdgeLabel(1).From=1;
EdgeLabel(1).To=-1;
EdgeLabel(1).Path=[];
EdgeLabelcnt=1;

mapread=map;

PrevNode=1;
totaliter=0;

plot_counter = 0;

%% 
% All the pixel nodes will be covered in this loop
while(1)
    
%     % read a windwos (3*3)
    temp=mapread(i-1:i+1,j-1:j+1);
    tempfindone=map(i-1:i+1,j-1:j+1);
    tempfindone(2,2)=0;

    
    if (mysum(PATTERN,temp)==0) % match with one of the patterns 
        
        if (search(i,j,mpstack) == -1)
            
            flag_min = 1;
            MIN_DIST = 6;
%             fprintf('length(EdgeLabel) = %d\n', length(EdgeLabel))
            for kk = 1:1:length(EdgeLabel)
                tam = size(EdgeLabel(kk).Path);
                tam = tam(1);
                if tam > 1
                    if (norm([i,j]-EdgeLabel(kk).Path(1,:)) < MIN_DIST)
                        flag_min = 0;
                    end
                end
                if tam > 2
                    if (norm([i,j]-EdgeLabel(kk).Path(end,:)) < MIN_DIST)
                        flag_min = 0;
                    end
                end
            end
            
            if (flag_min == 1)%ADRIANO
            
                totaliter=totaliter+size(TempPath,1);
                AddVer2G(i,j,vertexnum,0,totaliter);
                AddEdgeLabel(EdgeLabelcnt,TempPath,vertexnum,mpstack(top).vertexnum);%PrevNode);
                fprintf('add node(%d) \n',vertexnum);
                %connecting
                Conn2Ver2(mpstack,top,i,j,vertexnum,TempPath);
                TempPath=[];
                pathcounter=1;
                PrevNode=vertexnum;
                EdgeLabelcnt=EdgeLabelcnt+1;
                %cleanning stack
                DelStack(top);
                %adding to stack
                Add2Stack(top,i,j,vertexnum);
                top=top+1;
                Add2Stack(top,i,j,vertexnum);

                if (sum(sum(abs(PATTERN(7).x-temp)))==0 || sum(sum(abs(PATTERN(8).x-temp)))==0)
                    top=top+1;
                    Add2Stack(top,i,j,vertexnum)
                end

                vertexnum=vertexnum+1;
                map(i,j)=0;
            
            end%ADRIANO
            
        end  % end for search%
    end % if sum==0
    temp(2,2)=0;
    
    %-------------------------
    % if reach to end of the path
    if (sum(sum(tempfindone))==0)
        
        index=search(i,j,mpstack);
        
        if(index ~=-1)
            
            %connecting to index
            totaliter=totaliter+size(TempPath,1);
            Conn2Ver2(mpstack,index,mpstack(top).x,mpstack(top).y,mpstack(top).vertexnum,TempPath);
            AddEdgeLabel(EdgeLabelcnt,TempPath,mpstack(index).vertexnum,mpstack(top).vertexnum);
            EdgeLabelcnt=EdgeLabelcnt+1;
            TempPath=[];
            pathcounter=1;
            
            %cleanstack
            DelStack(index);
            i=mpstack(top).x;
            j=mpstack(top).y;
            
            if (i==0 || j==0)
                while(top>1)
                    if(mpstack(top).vertexnum~=0)
                        break;
                    end
                    top=top-1;
                end
                i=mpstack(top).x;
                j=mpstack(top).y;
            end
            
            %cleanstack
            DelStack(top);
            top=top-1;
            map(i,j)=0;
            iter=iter+1;
        else %else if index ==-1 there is no match point in stack
            
            if(searchVer(i,j,vertx) == -1)
%             if(searchVer(i,j,vertx) == -1)%ADRIANO
%                 flag_min = 1;
%                 MIN_DIST = 6;
%                 for kk = 1:1:length(EdgeLabel)
%                     if (norm([i,j]-EdgeLabel(kk).Path(1,:)) < MIN_DIST)
%                         flag_min = 0;
%                     end
%                     if (norm([i,j]-EdgeLabel(kk).Path(end,:)) < MIN_DIST)
%                         flag_min = 0;
%                     end
%                 end
%                 
%                 if (flag_min == 1)%ADRIANO

                    totaliter=totaliter+size(TempPath,1);
                    AddVer2G(i,j,vertexnum,0,totaliter);
                    AddEdgeLabel(EdgeLabelcnt,TempPath,vertexnum,mpstack(top).vertexnum);             
                    EdgeLabelcnt=EdgeLabelcnt+1;
                    fprintf('add node(%d) \n',vertexnum);
                    %connecting
                    Conn2Ver2(mpstack,top,i,j,vertexnum,TempPath);
                    TempPath=[];
                    pathcounter=1;
                    vertexnum=vertexnum+1;
                    i=mpstack(top).x;
                    j=mpstack(top).y;
                    PrevNode=mpstack(top).vertexnum;
                    %deleting stack
                    DelStack(top);
                    top=top-1;
                    map(i,j)=0;
                    iter=iter+1;

                else % impossible for move

                    while(top>1)
                        if(mpstack(top).vertexnum~=0)
                            break;
                        end
                        top=top-1;
                    end

                    if (top==0 || mpstack(top).x==0 || mpstack(top).y==0)
                        disp('finish');
                        break;
                    end

                    i=mpstack(top).x;
                    j=mpstack(top).y;
                    iter=iter-1;
                    TempPath=[];
                    pathcounter=1;
                end % end if, else
%             end%ADRIANO
         
        end % end if(index ~=-1)
    else %continue the path - if it is not end of the path
        
        map(i,j)=0;
        
        if (iter>=2)
            mapread(zi,zj)=0;
        end
        
        zi=i;
        zj=j;
        % first search in 4 main neighbors
        sumvec(1,:)=[-1  0 0 1]; % for x, row or i
        sumvec(2,:)=[ 0 -1 1 0]; % for y, col or j
        temp4Neigh=[tempfindone(1,2) tempfindone(2,1) tempfindone(2,3) tempfindone(3,2)];
        [k l]=find(temp4Neigh==1);
        
        if (size(k,2)>0)
            i=i+sumvec(1,l(1));%-2;
            j=j+sumvec(2,l(1));%-2;
        else
            tempDiagNeigh=[tempfindone(1,1) tempfindone(1,3) tempfindone(3,1)  tempfindone(3,3)];
            sumvecD(1,:)=[-1 -1  1 1 ]; % for x, row or i
            sumvecD(2,:)=[-1  1 -1 1]; % for y, col or j
            [k l]=find(tempDiagNeigh==1);
            if (size(k,2)>0)
                i=i+sumvecD(1,l(1));%-2;
                j=j+sumvecD(2,l(1));%-2;
            else
                printf ('no more unvisited path cell \n');
                break;
            end
        end
        iter=iter+1;
    end %if (sum(sum())==0
    
    TempPath(pathcounter,1:2)=[i j];
    pathcounter=pathcounter+1;
    showimg2(i-1:i+1,j-1:j+1)=5;
    
    plot_counter = plot_counter +1;
    if SHOWGVDFLAG && mod(plot_counter,20) == 0
        imagesc(showimg2); % for showing the matrix as an image
%         pause(0.0000001);
        drawnow
    end
    
end  %% end of while