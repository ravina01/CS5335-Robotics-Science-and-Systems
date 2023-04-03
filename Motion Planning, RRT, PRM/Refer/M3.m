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
% [samples, adjacency] = ex2_motion(2);
% ex2_motion(3, samples, adjacency);

    path_found =1;
    %path(1,1:4) = q_start
    n=1;
    y=1;
    path(1,1:4) = q_start;
    %explored(1) = 1;
    r1 = robot.fkine(q_start).t;
    r1 = r1(:,1)';
    d2 =1
    % connecting q start to the map
    for t = 1: length(samples)
       source =  q_start ; 
       neighbor = samples (t,1:4);
       num_collision =0;
       col_samples = M1(source, neighbor, 10);
      
      for s=1:length(col_samples)
        if check_collision(robot, col_samples(s,:), link_radius, sphere_centers, sphere_radii)
        num_collision = num_collision + 1;
        end
      end
        
        if(num_collision == 0)
            fprintf('M3 start near :');
            path(2,1:4) = samples (t,1:4)
            n=2;
            explored(1) = t
            t =length(samples)+1; 
            break;
        end
    end
    g=1;
       % connecting q goal to the map
     for t = 1: length(samples)
       source =  q_goal ; 
       neighbor = samples (t,1:4);
       num_collision =0;
       col_samples = M1(source, neighbor, 50);
      
      for s=1:length(col_samples)
        if check_collision(robot, col_samples(s,:), link_radius, sphere_centers, sphere_radii)
        num_collision = num_collision + 1;
        end
      end
        
        if(num_collision == 0)
            fprintf('M3 goal is near :');
            e_goal(g) = t; %consider all nodes connected to the goal for better performance
            g=g+1;
%             t =length(samples)+1; 
%             break;
        end
    end
e_goal
br= 0
%     path(2,1:4) = samples (2,1:4);
%     explored(2) = 2;n=n+1;y=y+1;
       for i=1:length(samples) % run for 10 pos
            if(br == 0)
         for j=1: length(samples)
             ex=0;
            for e = 1: n-1
                if(explored(e) == j) % avoid the xplored set
                    ex=1;
                end
            end
            if(adjacency(explored(y),j)>0) && (ex==0)  && (adjacency(explored(y),j)~=1)
                n=n+1;
                path(n,1:4) = samples (j,1:4);
                y=y+1;
                explored(y) = j;
                if(sum(j == e_goal))
                    fprintf('GOAAAAL:');
                    path_found =1;
                    br=1;
                    path(n+1,1:4) = q_goal;
                    i= length(samples)+1; %exit for loop
                    j= length(samples)+1;
                    break;
                end
                j= length(samples)+1;
                break;
            end
         end 
            end
       end
    path
end