% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
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

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)

%Lets initialize step size(alpha), starting node in tree (parent node)
tree = q_start;
path = [];
parents_node = 0;
ALPHA = 0.1; % started with step_size / alpha = 0.05 but was unable to find the path, hence decided to increase it's value.
             %  Feel free to modify alpha value in order to get results. 

% will make sure the path finding will stop when q_goal == nearest_sample or max iteration limit is reached. 
iterations_max_limit = 2000;


goal_reached = false;

for i = 1:iterations_max_limit
    
    % following the same psuedo code shared by prof. here, qtarget = q_rand_target
    q_rand_target = q_min + (q_max - q_min) .* rand(1,4);
    
    % Find the nearest configuration in the tree to the random configuration
    nearest_position = knnsearch(tree, q_rand_target);
    q_near = tree(nearest_position, :);
    
    % Generate a new configuration by moving towards the random configuration from the nearest configuration
    
    q_new = q_near + (ALPHA/norm(q_rand_target - q_near) * (q_rand_target - q_near));
    
    % You can either use check_collision / check_edge function to get
    % collision free path
    %Uncomment this line to use check_colliosn instead of check_edge
    %collision = check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii);
    
    check_edge_flag = check_edge(robot, q_new, q_near , link_radius, sphere_centers, sphere_radii);

    if ~check_edge_flag
        tree = [tree; q_new];

        % storing the parent, chlid node everytime, a new collison- free
        % node is added 
        parents_node = [parents_node; nearest_position];
        
        euclidean_distance = norm(q_new - q_goal);

        % Initially I was comparing the value of L2 distance with step_size, but path was not found. Hence, I increased this value.
        % I consider 0.6 as my goal tolerance, If euclidean distance
        % between q-new and q_goal is less than this tolerance, I consider
        % that I was successfully able to find the path
        if  euclidean_distance < 0.6
            goal_reached = true;
            break;
        end
    end
    if (goal_reached || i == iterations_max_limit)
       break
    end
end

% I had to do lot of experiments in order to backtrace all the node from q_goal to q_start and recover the path
if goal_reached
    path_found = true;
    path = q_goal;
    parent = parents_node(end);
    while parent ~= 1
        path = [tree(parent,:); path];
        parent = parents_node(parent);
    end
    path = [q_start; path];
    else
    path_found = false;
    path = [];
end

end


% If a path has been found, traverse the tree to recover the actual path
% if path_found
%     path = q_goal;
%     q = size(tree, 1);
%     while q ~= 1
%         q = parent(q);
%         path = [tree(q, :); path];
%     end
% else
%     path = [];
% end
% 
% end

%}