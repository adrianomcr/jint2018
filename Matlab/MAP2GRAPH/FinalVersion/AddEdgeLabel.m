function  AddEdgeLabel(EdgeLabCnt,TempPath,vertexnum,PrevNode)
    global EdgeLabel;
    EdgeLabel(EdgeLabCnt).From=PrevNode;
    EdgeLabel(EdgeLabCnt).To=vertexnum;
    EdgeLabel(EdgeLabCnt).Path=TempPath;
end
