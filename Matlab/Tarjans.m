



function [CG] = Tarjans(BM)

    
    if (length(BM(1,:)) ~= length(BM(:,1)))
        error('Matrix is not square !!!')
    end
    N = length(BM);
    
    
    placed = zeros(1,N);
    
    CG = struct('robots',[]);
    
    cluster = 0;
    
    for k = 1:1:N
       
        if placed(k) == 0
            C = search_BFS_mat(k, BM);
            cluster = cluster + 1;
            CG(cluster).robots = sort(C);
            
            for l = C
                placed(l) = 1;
            end
            
        end
        
        
    end





end