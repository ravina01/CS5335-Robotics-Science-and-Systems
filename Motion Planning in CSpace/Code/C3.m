% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    
    [n, m] = size(cspace);
    % Creates array of all Inf values
    distances = inf(n, m); 
    [~, goal_index_1] = min(abs(q_grid - q_goal(1)));
    [~, goal_index_2] = min(abs(q_grid - q_goal(2)));
    distances(goal_index_1, goal_index_2) = 2;

    % 8 neighbors in the grid
    neighbors = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];

    % goal configuration initialized to queue
    queue = [goal_index_1, goal_index_2];

    while ~isempty(queue)
        % Let's take first points in queue and then remove it from the
        % queue
        current = queue(1, :);
        queue(1, :) = [];
        
        % check the neighbors of the current configuration
        for i = 1:8 
            next = current + neighbors(i, :);
            x = next(1);
            y = next(2);
            if x >= 1 && x <= n && y >= 1 && y <= m
                if distances(x, y) == inf && cspace(x, y) == 0
                    distances(x, y) = distances(current(1), current(2)) + 1;
                    % adding next to the queue
                    queue = [queue; next];
                end
            end
        end
    end

    % set unreachable cells to 0 and obstacles to 1
    distances(isinf(distances)) = 0;
    distances(cspace == 1) = 1;

end
