% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)

    [n, m] = size(distances);
    [~, start_index_1] = min(abs(q_grid - q_start(1)));
    [~, start_index_2] = min(abs(q_grid - q_start(2)));

    % start configuration's grid cell
    path = [start_index_1, start_index_2];

    current = [start_index_1, start_index_2];
    while distances(current(1), current(2)) ~= 2
        % 8 neighbors in the grid
        neighbors = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];

        min_distance = inf;
        next = current;
        
        for i = 1:8
            neighbor = current + neighbors(i, :);
            x = neighbor(1);
            y = neighbor(2);
            if x >= 1 && x <= n && y >= 1 && y <= m
                if distances(x, y) == 2 
                    next = neighbor;
                    break 
                elseif distances(x, y) < min_distance && distances(x, y) > 1
                    min_distance = distances(x, y);
                    next = neighbor;
                end
            end
        end
        %Unable to find path condition
        if isequal(current, next)
            error('Unable to find a path to the goal');
        end
        current = next;
        %here, added the next grid cell to the path
        path = [path; current];
    end

end