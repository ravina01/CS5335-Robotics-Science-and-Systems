% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)


% Find the nearest vertex to the start configuration
start_position= knnsearch(samples, q_start);

% Find the nearest vertex to the goal configuration
goal_position = knnsearch(samples, q_goal);

[row, col] = size(adjacency);
% s= [];
% t = [];
% weights = [];
% for i = 1: row
%     s(end + 1) = i;
%     for j = 1 : col
%         t(end + 1) = j;
%         weights(end+1) = adjacency(i,j);
% 
%     end
% end
% G = graph(s, t, weights);

% Convert the adjacency matrix into a graph object
[s, t, weights] = find(adjacency);
G = graph(s, t, weights);

% Find the shortest path between the nearest vertices
[shortest_path, distances, path_found] = shortestpath(G, start_position, goal_position);

% Check for collisions along the path
if path_found
    path = samples(shortest_path, :);
    for iter = 1:size(path, 1)-1
        %q_path = path(iter, :);
        %if check_collision(robot, q_path, link_radius, sphere_centers, sphere_radii)
        if check_edge(robot, path(iter, :), path(iter+1, :), link_radius, sphere_centers, sphere_radii)
            path_found = false;
            path = [];
            break;
        end
    end
else
    path = [];
end
% before adding this piece of code, the path was stopping just one node
% before q_goal
path = [path; q_goal];
end