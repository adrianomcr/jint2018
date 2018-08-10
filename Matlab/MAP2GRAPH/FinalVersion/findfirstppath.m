function [i j]=findfirstppath(map)
[x,y]=size(map);
flag=0;
i=-1;
j=-1;
for m=2:x-1
    if (flag==1)
        break;
    end
    for n=2:y-1
        if(map(m,n)==1)
            flag=1;
            i=m;
            j=n;
            break;
        end        
    end
end