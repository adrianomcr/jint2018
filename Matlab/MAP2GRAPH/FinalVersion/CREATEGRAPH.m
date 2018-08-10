function [Path MeetPoints]=CREATEGRAPH(EdgeLabel,vertex)
n1=1;


[m n]=size(EdgeLabel);

for i=1:n
    aa=EdgeLabel(i).Path;
    n2=size(aa,1);
    %aa(:,3)=n1:n1+n2-1;
    IndexL(1:2,i)=[n1 n1+n2-1];
    Path(i).path=aa;
    Path(i).from=EdgeLabel(1,i).From;
    Path(i).To=EdgeLabel(1,i).To;
    n1=n1+n2;
end


[l k]=size(vertex);

for i=1:k
    MeetPoints(i).x=vertex(1,i).x;
    MeetPoints(i).y=vertex(1,i).y;
    MeetPoints(i).indexinPath= vertex(1,i).IndexInPath;
end

end
