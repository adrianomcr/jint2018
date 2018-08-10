function out=searchVer(i,j,vertx) 
len=size(vertx,2);
out=-1;
for k=1:len
    if ((vertx(k).x==i || vertx(k).x==i-1 || vertx(k).x==i+1)... 
            && (vertx(k).y==j || vertx(k).y==j-1 || vertx(k).y==j+1))
        out=k;
        break;
    end
end
