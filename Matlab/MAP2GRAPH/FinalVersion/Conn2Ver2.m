function    Conn2Ver2(mpstack,top,i,j,vertexnum,path)
global vertx;
vertx(1,mpstack(top).vertexnum).nodnum=vertx(1,mpstack(top).vertexnum).nodnum+1;
nodnum=vertx(1,mpstack(top).vertexnum).nodnum;
vertx(1,mpstack(top).vertexnum).node(nodnum).x=i;
vertx(1,mpstack(top).vertexnum).node(nodnum).y=j;
vertx(1,mpstack(top).vertexnum).node(nodnum).vertexnum=vertexnum;
vertx(1,mpstack(top).vertexnum).node(nodnum).path=path;
end
