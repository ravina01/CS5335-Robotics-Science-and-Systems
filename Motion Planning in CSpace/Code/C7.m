% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = C7(cspace)
padded_cspace = cspace;
for i = 2:length (cspace)-1
    for j = 2:length(cspace)-1
        if(cspace(i,j)==0)
            if(cspace(i-1,j-1) == 1)
                padded_cspace(i,j) = 1;
            elseif(cspace(i-1,j) == 1)
                padded_cspace(i,j) = 1;
            elseif(cspace(i-1,j+1) == 1)
                padded_cspace(i,j) = 1; 
            elseif(cspace(i,j-1) == 1)
                padded_cspace(i,j) = 1;
            elseif(cspace(i,j+1)==1)
                padded_cspace(i,j) = 1;
            elseif(cspace(i+1,j-1)==1)
                padded_cspace(i,j) = 1;
            elseif(cspace(i+1,j)==1)
                padded_cspace(i,j) = 1;
            elseif(cspace(i+1,j+1)==1)
                padded_cspace(i,j) = 1;
            else % do nothing
                padded_cspace(i,j) = 0;
            end
        end
    end
end
end