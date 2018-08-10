function out=mysum(t,temp)
out=1;
len=size(t,2);
for i=1:len
    if(sum(sum(abs(t(i).x-temp)))==0)
        out=0;
        break;
    end
end
end
