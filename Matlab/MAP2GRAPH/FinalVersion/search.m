function out=search(i,j,mpstack)
len=size(mpstack,2);
out=-1;
for k=1:len
    if(abs(mpstack(k).x-i)<=1 && abs(mpstack(k).y-j)<=1)
%     if ((mpstack(k).x==i || mpstack(k).x==i-1 || mpstack(k).x==i+1)... 
%             && (mpstack(k).y==j || mpstack(k).y==j-1 || mpstack(k).y==j+1))
        out=k;
        break;
    end
end

 