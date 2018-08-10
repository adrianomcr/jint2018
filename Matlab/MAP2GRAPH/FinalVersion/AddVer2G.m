function AddVer2G(i,j,vertexnum,nodenum,indexinpath)
global vertx;
vertx(1,vertexnum).x=i;
vertx(1,vertexnum).y=j;
vertx(1,vertexnum).vertexnum=vertexnum;
vertx(1,vertexnum).nodnum=nodenum;
vertx(1,vertexnum).IndexInPath=indexinpath;
end
