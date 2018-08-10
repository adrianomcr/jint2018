%Universidade Federal de Minas Gerais - 2018/1
%Jint 2018
%Adriano Martins da Costa Rezende

function [C] = search_BFS_mat(v, BM)

    %v - noh inicial
    %BM - especificacao do grafo in binary matrix form
    %OBS: no campo children_set de 'S' nao estao os filhos, mas sim os vizinhos!!
    
    
    if (length(BM(1,:)) ~= length(BM(:,1)))
        error('Matrix is not square !!!')
    end
    N = length(BM);
    
    
    %Add v as a neighbor of v itself
    BM(v,v) = 1;
    
    % coloca v em O
    O=[v v]; %[noh, noh pai]
    % Inicia o conjunto fechado C
    C=[];
     
    % enquanto a fila esta vazia
    while length(O(:,1)) > 0
        % remove u de O
        u=O(1,1);
        pu=O(1,2); % parent of u
        O(1,:)=[];
        
        % Se u nao estiver em C coloque u em C
        if length(find(C==u))==0
            % coloque u em C
            C(end+1)=u;
            
            
            % Coloque os vizinhos do u que nao estao em C em O
            for i = 1:1:N %for em todos os nos
                if BM(u,i) == 1 %se eh vizinho
                    if length(find(C==i))==0
                        O(end+1,1) = i;
                        O(end,2) = u;
                    end
                end
            end


                     
        end 
        
    end

end