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
Tree(1,1) = 0
nodes(1,1:4) = q_start;
num_samples=10

%%%ADD LEAVES%%%
 N= rand(num_samples); %N random values, 0 to 1
%     N = N';
%     N =sort(N);
    num_collision = 0;
    i=2;
%     samples(1,1:4)= [0 -pi/4 0 -pi/4];
%     samples(2,1:4)= [0 -3 0 -3];
%     i=3;
n=1;
    while(i<=num_samples)
        consider_q = q_min(1:4) + ((q_max(1:4)-q_min(1:4)).*N(n,1:4));
        n= n+1;
        if(n>num_samples)
            n=1;
        end
      if check_collision(robot, consider_q, link_radius, sphere_centers, sphere_radii,100)
                num_collision = num_collision + 1;
      else
          nodes(i,1:4)= consider_q;
          d = (robot.fkine(q_goal).t - robot.fkine(consider_q).t).^2; %distance to goal
          d =sqrt(sum(d));
          Tree(1,i) = d;
          i = i+1;
     end
    end

    %%%%%%%BAKAR
    path_found = 1
       x=[0.3; 0.3; 0.3]';
       t= trotz(0);
       t(1:3,4) = x
       q = robot.ikine(t,'mask', [1, 1, 0, 1, 0, 0])
       path(1,:) = q_start
       path(2,:) = q
       path(3,:) = q_goal
       Tree
end