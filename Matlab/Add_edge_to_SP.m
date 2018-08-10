

clear all
M = dlmread('./../distributed/maps/Map_36_SP_BAK.txt');


% SP = [];
load('Edge_SP.mat','SP')

M = [M,SP];

FILE = fopen('./../distributed/maps/Map_36_SP.txt','w');
for k = 1:1:length(M(:,1))
    fprintf(FILE,'%.4f \t %.4f \t %d\n',M(k,1),M(k,2),M(k,3))
%     fprintf(FILE,'%.4f \t %.4f\n',M(k,1),M(k,2));
end
close(FILE)
%dlmwrite('./../distributed/maps/Map_36_SP.txt',M)

% 
% 
% FILE = fopen('./../distributed/maps/Map_36_SP.txt','w');
% for k = 1:1:length(Mnew(:,1))
%     fprintf(FILE,'%.4f \t %.4f \t %d\n',Mnew(k,1),Mnew(k,2),Mnew(k,3))
% %     fprintf(FILE,'%.4f \t %.4f\n',M(k,1),M(k,2));
% end
% close(FILE)
% %dlmwrite('./../distributed/maps/Map_36_SP.txt',M)